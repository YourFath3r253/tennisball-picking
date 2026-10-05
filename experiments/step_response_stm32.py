"""原地轉向步階響應，走跟實體車一模一樣的資料流 (2026-10-05 數位孿生介面)：

  Gazebo 相機 -> vision_node (真實相機模型 + D 同學 UART 協定 BALL,距離,角度，EMA 0.35)
  -> 本程式 = 實體車 STM32 的 Chassis_SteerP：y = Kp*(0-角度)，左 -y/2、右 +y/2 (飽和 ±45 rpm)
  -> /wheel_target_rpm -> motor_driver_node (STM32 輪速 PID + 馬達模型) -> Gazebo

Kp 單位跟實體車 main.c 的 STEER_KP 一樣 (rpm/度)，所以可以直接跟實體車 run12~15 比。
指標跟分析實體車 CSV 用同一套 (fit_motor_model.metrics)：開始轉 -> 進入 ±4° 的時間、overshoot、
擺盪時的等效延遲 (衝過頭角度 / 過零角速度)、擺幅。

用法 (Gazebo + vision_node 要先開著，標準 9 顆球 world；motor_driver_node 由 sim_launch 帶起來)：
  python3 experiments/step_response_stm32.py <out.csv> <Kp> [bearing_deg=29] [duration=20] [comp=off|gyro|encoder] [D=0.8]
相機延遲補償 (comp)：見 experiments/delay_comp_design.py。收到 BALL 時目標航向 psi* = psi(現在-D) - 角度，
之後每 10 ms 用最新轉角 (陀螺儀 /imu 或編碼器積分 /wheel_rpm) 算誤差 e = psi* - psi，y = Kp*e。
"""
from collections import deque
import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String
from gazebo_msgs.srv import GetEntityState, SetEntityState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fit_motor_model import metrics  # noqa: E402

PX_PER_RAD = 458.5
ROBOT_X, ROBOT_Y = -6.0, 0.0
CAM_IN_BASE = (0.10, -0.135)
BALL_DIST = 0.8
STEER_WHEEL_MAX = 45.0
WHEEL_R, WHEEL_L = 0.052, 0.243  # 實體車幾何 (main.c)，編碼器積分轉角用
OTHER_BALLS = ['ball_2', 'ball_3', 'ball_4', 'ball_5', 'ball_7', 'ball_8', 'ball_9', 'ball_10', 'ball_6']


class StepTest(Node):
    def __init__(self, out, kp, bearing_deg, duration, comp='off', delay=0.8):
        super().__init__('step_response_stm32')
        self.kp, self.duration = kp, duration
        self.comp, self.delay = comp, delay
        self.sim_now = None
        self.gyro_yaw = 0.0      # deg，逆時針正
        self.enc_yaw = 0.0
        self._last_imu = None
        self._last_rpm_t = None
        self.hist = deque(maxlen=3000)  # (sim_t, gyro_yaw, enc_yaw)
        self.psi_target = None
        self.create_subscription(Imu, '/imu', self._imu_cb, 50)
        self.create_timer(0.01, self._comp_tick)
        self.bearing0 = math.radians(bearing_deg)
        self.wheel_pub = self.create_publisher(Float32MultiArray, '/wheel_target_rpm', 10)
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_cli.wait_for_service()
        self.set_cli.wait_for_service()
        self.raw = None          # 原始角度 (右邊為正)，對應實體車 CSV 的 bearing_deg
        self.raw_new = False
        self.uart = None
        self.rpm = None
        self.armed = False
        self.create_subscription(Point, '/target_position', self._target_cb, 10)
        self.create_subscription(String, '/jetson_uart', self._uart_cb, 10)
        self.create_subscription(Float32MultiArray, '/wheel_rpm', self._rpm_cb, 10)
        self.f = open(out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'raw_bearing_deg', 'uart_bearing_deg', 'true_bearing_deg', 'new_vision',
                         'target_left', 'actual_left', 'target_right', 'actual_right'])

    def _imu_cb(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self._last_imu is not None and 0 < st - self._last_imu < 0.5:
            self.gyro_yaw += math.degrees(m.angular_velocity.z) * (st - self._last_imu)
        self._last_imu = st
        self.sim_now = st
        self.hist.append((st, self.gyro_yaw, self.enc_yaw))

    def _rpm_cb(self, m):
        self.rpm = list(m.data)
        if self.sim_now is None:
            return
        if self._last_rpm_t is not None and 0 < self.sim_now - self._last_rpm_t < 0.5:
            wl = m.data[1] * 2 * math.pi / 60.0
            wr = m.data[3] * 2 * math.pi / 60.0
            self.enc_yaw += math.degrees(WHEEL_R * (wr - wl) / WHEEL_L) * (self.sim_now - self._last_rpm_t)
        self._last_rpm_t = self.sim_now

    def _meas(self, row=None):
        if row is None:
            return self.gyro_yaw if self.comp == 'gyro' else self.enc_yaw
        return row[1] if self.comp == 'gyro' else row[2]

    def _yaw_at(self, t):
        val = None
        for row in self.hist:
            if row[0] <= t:
                val = self._meas(row)
            else:
                break
        return self._meas() if val is None else val

    def _comp_tick(self):
        if not self.armed or self.comp == 'off' or self.psi_target is None:
            return
        y = self.kp * (self.psi_target - self._meas())
        half = max(-STEER_WHEEL_MAX, min(STEER_WHEEL_MAX, y / 2.0))
        self._wheels(-half, half if half != 0.0 else 1e-6)

    def _target_cb(self, m):
        if m.z == 1.0:
            self.raw = math.degrees((m.x - 320.0) / PX_PER_RAD)
            self.raw_new = True

    def _uart_cb(self, m):
        line = m.data
        if not self.armed:
            return
        if line.startswith('BALL,'):
            ang = float(line.split(',')[2])
            self.uart = ang
            if self.comp != 'off' and self.sim_now is not None:
                self.psi_target = self._yaw_at(self.sim_now - self.delay) - ang
                return
            y = self.kp * (0.0 - ang)
            half = max(-STEER_WHEEL_MAX, min(STEER_WHEEL_MAX, y / 2.0))
            self._wheels(-half, half)
        elif line.startswith('BALL_OFF'):
            self.psi_target = None
            self._wheels(0.0, 0.0)

    def _wheels(self, l, r):
        msg = Float32MultiArray()
        msg.data = [float(l), float(r)]
        self.wheel_pub.publish(msg)

    def call(self, cli, req):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return f.result()

    def set_pose(self, name, x, y, z, yaw=0.0):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, z
        req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        req.state.reference_frame = 'world'
        self.call(self.set_cli, req)

    def robot_pose(self):
        req = GetEntityState.Request()
        req.name = 'tennis_bot::base_link'
        r = self.call(self.get_cli, req)
        o = r.state.pose.orientation
        return r.state.pose.position, math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))

    def run(self):
        self._wheels(0.0, 0.0)
        for i, name in enumerate(OTHER_BALLS):
            self.set_pose(name, -11.5, -5.0 + i * 1.2, 0.033)
        self.set_pose('tennis_bot', ROBOT_X, ROBOT_Y, 0.05, 0.0)
        cam_x, cam_y = ROBOT_X + CAM_IN_BASE[0], ROBOT_Y + CAM_IN_BASE[1]
        # 球在右邊 (+bearing) = 車體座標 y 負方向
        ball_x = cam_x + BALL_DIST * math.cos(-self.bearing0)
        ball_y = cam_y + BALL_DIST * math.sin(-self.bearing0)
        self.set_pose('ball_1', ball_x, ball_y, 0.033)
        t_end = time.time() + 3.0
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.armed = True
        t0 = time.time()
        while time.time() - t0 < self.duration:
            rclpy.spin_once(self, timeout_sec=0.02)
            p, yaw = self.robot_pose()
            cx = p.x + CAM_IN_BASE[0] * math.cos(yaw) - CAM_IN_BASE[1] * math.sin(yaw)
            cy = p.y + CAM_IN_BASE[0] * math.sin(yaw) + CAM_IN_BASE[1] * math.cos(yaw)
            true_b = -math.degrees(math.atan2(ball_y - cy, ball_x - cx) - yaw)
            rpm = self.rpm or [0, 0, 0, 0]
            self.w.writerow([f'{time.time() - t0:.3f}', '' if self.raw is None else f'{self.raw:.2f}',
                             '' if self.uart is None else f'{self.uart:.2f}', f'{true_b:.2f}', int(self.raw_new),
                             f'{rpm[0]:.2f}', f'{rpm[1]:.2f}', f'{rpm[2]:.2f}', f'{rpm[3]:.2f}'])
            self.raw_new = False
        self.armed = False
        self._wheels(0.0, 0.0)
        self.f.close()


def analyze(path):
    rows = [r for r in csv.DictReader(open(path)) if r['new_vision'] == '1' and r['raw_bearing_deg']]
    return metrics([float(r['t']) for r in rows], [float(r['raw_bearing_deg']) for r in rows])


def main():
    out, kp = sys.argv[1], float(sys.argv[2])
    bearing = float(sys.argv[3]) if len(sys.argv) > 3 else 29.0
    duration = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
    comp = sys.argv[5] if len(sys.argv) > 5 else 'off'
    delay = float(sys.argv[6]) if len(sys.argv) > 6 else 0.8
    rclpy.init()
    node = StepTest(out, kp, bearing, duration, comp, delay)
    node.run()
    rclpy.shutdown()
    m = analyze(out)
    fmt = lambda v: '-' if v is None else f'{v:.2f}'
    print(f'Kp={kp} comp={comp} D={delay}: rise(±4°) {fmt(m["rise"])} s, overshoot {fmt(m["over"])}°, '
          f'等效延遲 {fmt(m["tau"])} s, 擺幅 {fmt(m["amp"])}°', flush=True)


if __name__ == '__main__':
    main()
