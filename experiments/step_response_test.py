"""原地轉向步階響應 (模擬版的實體車 run12~15 實驗)：用來校正/驗證相機延遲模型。

車停在原地，球放在相機視野一側 (預設 +28°、1.0 m)，只做原地轉向 P 控制
  ω = clamp(K * 像素誤差, ±max_omega)
記錄 vision 回報的球角度 (bearing) 跟 Gazebo 真實航向，算出跟實體車一樣的指標：
  - 等效延遲 tau_eff = 衝過頭的角度 / 衝過零點時的角速度 (跟分析 run12 用的方法完全一樣)
  - rise time (第一次進入 ±4°)、最大 overshoot

用法 (Gazebo + vision_node 要先開著，標準 9 顆球 world)：
  python3 experiments/step_response_test.py <out.csv> <K> [max_omega] [bearing_deg] [duration]
  例：python3 experiments/step_response_test.py /tmp/s.csv 0.02 0.6 28 12
"""
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.srv import GetEntityState, SetEntityState

PX_PER_RAD = 458.5
ROBOT_X, ROBOT_Y = -6.0, 0.0
CAM_IN_BASE = (0.10, -0.135)  # 相機在 base_link 座標 (front_chassis 轉 -90° 後)
BALL_DIST = 1.0
OTHER_BALLS = ['ball_2', 'ball_3', 'ball_4', 'ball_5', 'ball_7', 'ball_8', 'ball_9', 'ball_10', 'ball_6']


def yaw_of(o):
    return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))


class StepTest(Node):
    def __init__(self, out, k, max_omega, bearing_deg, duration):
        super().__init__('step_response_test')
        self.k, self.max_omega, self.duration = k, max_omega, duration
        self.bearing0 = math.radians(bearing_deg)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_cli.wait_for_service()
        self.set_cli.wait_for_service()
        self.target = None
        self.target_t = None
        self.create_subscription(Point, '/target_position', self.target_cb, 10)
        self.f = open(out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'measured_bearing_deg', 'true_bearing_deg', 'omega_cmd', 'new_vision'])

    def target_cb(self, m):
        self.target = m
        self.target_t = time.time()

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
        return r.state.pose.position, yaw_of(r.state.pose.orientation)

    def run(self):
        for i, name in enumerate(OTHER_BALLS):  # 其他球搬到車後方很遠的地方
            self.set_pose(name, -11.5, -5.0 + i * 1.2, 0.033)
        self.set_pose('tennis_bot', ROBOT_X, ROBOT_Y, 0.05, 0.0)
        cam_x, cam_y = ROBOT_X + CAM_IN_BASE[0], ROBOT_Y + CAM_IN_BASE[1]
        ball_x = cam_x + BALL_DIST * math.cos(self.bearing0)
        ball_y = cam_y + BALL_DIST * math.sin(self.bearing0)
        self.set_pose('ball_1', ball_x, ball_y, 0.033)
        t_end = time.time() + 3.0  # 等車停穩、視覺看到球
        while time.time() < t_end:
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

        t0 = time.time()
        last_target_t = None
        while time.time() - t0 < self.duration:
            rclpy.spin_once(self, timeout_sec=0.02)
            tw = Twist()
            meas = None
            if self.target is not None and self.target.z == 1.0:
                err_px = 320.0 - self.target.x
                meas = math.degrees(err_px / PX_PER_RAD)
                tw.angular.z = max(-self.max_omega, min(self.max_omega, self.k * err_px))
            self.cmd.publish(tw)
            p, yaw = self.robot_pose()
            cx = p.x + CAM_IN_BASE[0] * math.cos(yaw) - CAM_IN_BASE[1] * math.sin(yaw)
            cy = p.y + CAM_IN_BASE[0] * math.sin(yaw) + CAM_IN_BASE[1] * math.cos(yaw)
            true_b = math.degrees(math.atan2(ball_y - cy, ball_x - cx) - yaw)
            new = int(self.target_t is not None and self.target_t != last_target_t)
            last_target_t = self.target_t
            self.w.writerow([f'{time.time() - t0:.3f}', '' if meas is None else f'{meas:.2f}',
                             f'{true_b:.2f}', f'{tw.angular.z:.3f}', new])
        self.cmd.publish(Twist())
        self.f.close()


def analyze(path):
    rows = list(csv.DictReader(open(path)))
    # 只用「有新視覺資料」的那幾列，跟實體車 CSV 一樣是每次視覺輸出一筆
    vis = [(float(r['t']), float(r['measured_bearing_deg'])) for r in rows
           if r['new_vision'] == '1' and r['measured_bearing_deg'] != '']
    tru = [(float(r['t']), float(r['true_bearing_deg'])) for r in rows]
    out = {}
    for label, seq in (('measured', vis), ('true', tru)):
        if len(seq) < 3:
            continue
        T = [a for a, _ in seq]
        B = [b for _, b in seq]
        rise = next((t for t, b in seq if abs(b) <= 4.0), None)
        sign0 = 1 if B[0] > 0 else -1
        over = max((-sign0 * b for b in B), default=0.0)
        taus = []
        for i in range(1, len(B)):
            if (B[i - 1] > 0) != (B[i] > 0):
                j0 = max([k for k in range(i) if T[k] <= T[i - 1] - 0.6] or [0])
                speed = (B[i] - B[j0]) / (T[i] - T[j0]) if T[i] > T[j0] else 0
                sgn = 1 if B[i] > 0 else -1
                ext = 0.0
                for k in range(i, len(B)):
                    if T[k] - T[i] > 4 or (B[k] > 0) != (sgn > 0):
                        break
                    ext = max(ext, abs(B[k]))
                if abs(speed) > 1:
                    taus.append((round(T[i], 2), round(speed, 1), round(ext, 1), round(ext / abs(speed), 2)))
        out[label] = {'rise_s': rise, 'max_overshoot_deg': round(over, 2), 'swings': taus}
    return out


def main():
    out = sys.argv[1]
    k = float(sys.argv[2])
    max_omega = float(sys.argv[3]) if len(sys.argv) > 3 else 0.6
    bearing = float(sys.argv[4]) if len(sys.argv) > 4 else 28.0
    duration = float(sys.argv[5]) if len(sys.argv) > 5 else 12.0
    rclpy.init()
    node = StepTest(out, k, max_omega, bearing, duration)
    node.run()
    rclpy.shutdown()
    print(analyze(out))


if __name__ == '__main__':
    main()
