"""撿球機構效能實驗：滾輪角速度 (roller_omega) x 盲抓前進速度 (blind_speed)
對「10 顆隨機球全部撿完所需時間 / 漏球數」的影響。

流程（每個 (omega, blind_speed) 組合 x 3 組固定佈局各跑一次）：
  1. 用 /set_entity_state 把機器人歸位到原點、10 顆球傳送到該組佈局位置，速度歸零
  2. 用 /control_node/set_parameters 設定這次要測的 roller_omega, blind_speed
  3. 打開 /capture_mode，讓 control_node 開始 SEARCH -> TRACK -> BLIND_CAPTURE 循環
  4. 監看 /control_node/state；每次偵測到「離開 BLIND_CAPTURE」的瞬間，等 1 秒讓球
     物理沉澱，再用 /get_entity_state 查還沒被記錄成功的球，判斷是否落在後車籃
     (rear_basket) 的邊界內
  5. 全部 10 顆都判定完成，或超過單次 trial 時間上限，就結束並記錄結果

球是否「進籃」的判定：把球的世界座標轉換到機器人 base_link 局部座標系
(用 /get_entity_state 查到的機器人姿態做四元數旋轉)，再檢查是否落在依
rear_basket.stl 網格實際外緣量出來、外加一點誤差邊界的長方體範圍內。
"""
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState, SetEntityState
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Bool, String

from layouts import LAYOUTS, NUM_BALLS

# ---- 掃描參數範圍 ----
ROLLER_OMEGA_VALUES = [30.0, 35.0, 40.0, 45.0, 50.0]   # rad/s，30 以下拋不進籃
BLIND_SPEED_VALUES = [0.1, 0.2, 0.3, 0.4, 0.5]          # m/s

TRIAL_TIMEOUT_SEC = 300.0        # 單次 trial 時間上限 (真實秒數)
SETTLE_WAIT_SEC = 1.0            # 盲抓結束後等球physics穩定再判定
POLL_PERIOD_SEC = 0.15

# rear_basket 在 base_link 局部座標系的邊界框 (公尺)，從 rear_basket.stl 網格
# 實際頂點外緣量出來 (見對話紀錄)，再各邊加約 0.03m 誤差邊界
BASKET_LOCAL_BOUNDS = {
    'x': (-0.40, 0.03),
    'y': (-0.28, 0.01),
    'z': (0.00, 0.25),
}

RESULTS_CSV = Path(__file__).parent / 'results.csv'


def quat_rotate_inverse(qx, qy, qz, qw, vx, vy, vz):
    """把世界座標向量 v 轉到姿態為 (qx,qy,qz,qw) 的物體局部座標系
    (等於用共軛四元數對 v 做旋轉)。"""
    # conjugate
    cx, cy, cz, cw = -qx, -qy, -qz, qw
    # v as pure quaternion (0, v)
    # t = 2 * cross(q_vec, v)
    tx = 2.0 * (cy * vz - cz * vy)
    ty = 2.0 * (cz * vx - cx * vz)
    tz = 2.0 * (cx * vy - cy * vx)
    # v_rot = v + w*t + cross(q_vec, t)
    rx = vx + cw * tx + (cy * tz - cz * ty)
    ry = vy + cw * ty + (cz * tx - cx * tz)
    rz = vz + cw * tz + (cx * ty - cy * tx)
    return rx, ry, rz


class PickupExperiment(Node):
    def __init__(self):
        super().__init__('pickup_experiment')
        self.get_entity_state_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_entity_state_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.set_params_cli = self.create_client(SetParameters, '/control_node/set_parameters')
        self.capture_mode_pub = self.create_publisher(Bool, '/capture_mode', 10)
        self.create_subscription(String, '/control_node/state', self._state_cb, 10)

        self.last_control_state = None
        self.prev_control_state = None
        self.blind_capture_ended_at = None  # wall-clock time.time() when we last left BLIND_CAPTURE

        for cli, name in [
            (self.get_entity_state_cli, '/get_entity_state'),
            (self.set_entity_state_cli, '/set_entity_state'),
            (self.set_params_cli, '/control_node/set_parameters'),
        ]:
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'waiting for {name}...')

    def _state_cb(self, msg):
        self.prev_control_state = self.last_control_state
        self.last_control_state = msg.data
        if self.prev_control_state == 'BLIND_CAPTURE' and self.last_control_state != 'BLIND_CAPTURE':
            self.blind_capture_ended_at = time.time()

    def _spin_briefly(self, duration_sec):
        end = time.time() + duration_sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.1, end - time.time())))

    def get_entity_pose(self, name):
        req = GetEntityState.Request()
        req.name = name
        future = self.get_entity_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        if resp is None or not resp.success:
            return None
        p = resp.state.pose.position
        o = resp.state.pose.orientation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    def set_entity_pose(self, name, x, y, z, yaw=0.0):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0
        future = self.set_entity_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        return resp is not None and resp.success

    def set_control_params(self, roller_omega, blind_speed):
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='roller_omega',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(roller_omega))),
            Parameter(name='blind_speed',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(blind_speed))),
        ]
        future = self.set_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None

    def reset_trial(self, layout):
        # 機器人回到原點、速度歸零、朝向歸零
        self.set_entity_pose('tennis_bot', 0.0, 0.0, 0.05, yaw=0.0)
        for i, (x, y) in enumerate(layout):
            self.set_entity_pose(f'ball_{i + 1}', x, y, 0.1)
        self._spin_briefly(0.5)  # 讓物理沉澱一下

    def is_ball_in_basket(self, ball_name):
        ball_pose = self.get_entity_pose(ball_name)
        robot_pose = self.get_entity_pose('tennis_bot')
        if ball_pose is None or robot_pose is None:
            return False
        (bx, by, bz), _ = ball_pose
        (rx, ry, rz), (qx, qy, qz, qw) = robot_pose
        dx, dy, dz = bx - rx, by - ry, bz - rz
        lx, ly, lz = quat_rotate_inverse(qx, qy, qz, qw, dx, dy, dz)
        xr = BASKET_LOCAL_BOUNDS['x']
        yr = BASKET_LOCAL_BOUNDS['y']
        zr = BASKET_LOCAL_BOUNDS['z']
        return xr[0] <= lx <= xr[1] and yr[0] <= ly <= yr[1] and zr[0] <= lz <= zr[1]

    def run_trial(self, roller_omega, blind_speed, layout, layout_id):
        self.get_logger().info(
            f'--- trial omega={roller_omega} blind_speed={blind_speed} layout={layout_id} ---'
        )
        self.reset_trial(layout)
        self.set_control_params(roller_omega, blind_speed)

        self.last_control_state = None
        self.prev_control_state = None
        self.blind_capture_ended_at = None

        captured = [False] * NUM_BALLS
        capture_time = [None] * NUM_BALLS
        pending_check = False

        self.capture_mode_pub.publish(Bool(data=True))

        start = time.time()
        while time.time() - start < TRIAL_TIMEOUT_SEC:
            self._spin_briefly(POLL_PERIOD_SEC)

            if self.blind_capture_ended_at is not None:
                if not pending_check:
                    pending_check = True
                    check_at = self.blind_capture_ended_at + SETTLE_WAIT_SEC
                if time.time() >= check_at:
                    for i in range(NUM_BALLS):
                        if not captured[i] and self.is_ball_in_basket(f'ball_{i + 1}'):
                            captured[i] = True
                            capture_time[i] = time.time() - start
                            self.get_logger().info(f'  ball_{i + 1} captured at t={capture_time[i]:.1f}s')
                    self.blind_capture_ended_at = None
                    pending_check = False

            if all(captured):
                break

        self.capture_mode_pub.publish(Bool(data=False))
        total_time = time.time() - start
        miss_count = sum(1 for c in captured if not c)

        return {
            'roller_omega': roller_omega,
            'blind_speed': blind_speed,
            'layout_id': layout_id,
            'total_time': total_time,
            'miss_count': miss_count,
            'capture_times': list(capture_time),
        }


def main():
    rclpy.init()
    node = PickupExperiment()

    write_header = not RESULTS_CSV.exists()
    with open(RESULTS_CSV, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(['roller_omega', 'blind_speed', 'layout_id', 'total_time', 'miss_count',
                              'capture_times'])

        for omega in ROLLER_OMEGA_VALUES:
            for blind_speed in BLIND_SPEED_VALUES:
                for layout_id, layout in enumerate(LAYOUTS):
                    result = node.run_trial(omega, blind_speed, layout, layout_id)
                    writer.writerow([
                        result['roller_omega'], result['blind_speed'], result['layout_id'],
                        f"{result['total_time']:.2f}", result['miss_count'],
                        ';'.join('' if t is None else f'{t:.2f}' for t in result['capture_times']),
                    ])
                    f.flush()

    node.get_logger().info('全部組合跑完')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
