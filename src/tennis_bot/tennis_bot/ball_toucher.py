"""網球碰到「前車體 (front_chassis)」才消失，後車體/其他部位碰到不算。

Gazebo 裡 front_chassis 是用 fixed joint 掛在 base_link 下面，物理引擎會把
fixed joint 的子 link 直接併入父 link（效能考量），所以沒辦法直接查詢
"tennis_bot::front_chassis" 的座標 (試過，GetEntityState 對這個名字回
success=False)。改成用 base_link 的座標+姿態，把每顆球的世界座標轉到
base_link 局部座標系，再檢查是否落在 front_chassis 網格實際外緣量出來的
範圍內 (見對話紀錄的 front_chassis.stl bounding box)。

這個節點本身不會讓車體動——只負責監看+刪除，方便你自己用
teleop_twist_keyboard 手動開車去測試觸碰判定準不準。

技術細節：查詢座標用 call_async + add_done_callback 全非同步串接，不能在
timer callback 裡面用 spin_until_future_complete 等結果——那個 callback
本身就是外層 rclpy.spin(node) 呼叫出來的，在裡面再嵌一層 spin 等待，
會卡死或根本等不到回應 (實測真的會這樣，query 永遠沒有結果)。
"""
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState, DeleteEntity

NUM_BALLS = 10  # 目前世界裡的球數，跟 tennis_court.world 一致
BALL_NAME_PREFIX = 'ball_'
CHECK_PERIOD_SEC = 0.2

# front_chassis 在 base_link 局部座標系的觸碰範圍 (公尺)，從
# front_chassis.stl 網格實際外緣量出來、加一點誤差邊界
FRONT_TOUCH_X_RANGE = (-0.05, 0.35)
FRONT_TOUCH_Y_RANGE = (-0.35, 0.08)


def quat_rotate_inverse(qx, qy, qz, qw, vx, vy, vz):
    """把世界座標向量 v 轉到姿態為 (qx,qy,qz,qw) 的物體局部座標系。"""
    cx, cy, cz, cw = -qx, -qy, -qz, qw
    tx = 2.0 * (cy * vz - cz * vy)
    ty = 2.0 * (cz * vx - cx * vz)
    tz = 2.0 * (cx * vy - cy * vx)
    rx = vx + cw * tx + (cy * tz - cz * ty)
    ry = vy + cw * ty + (cz * tx - cx * tz)
    rz = vz + cw * tz + (cx * ty - cy * tx)
    return rx, ry, rz


class BallToucher(Node):
    def __init__(self):
        super().__init__('ball_toucher')
        self.get_entity_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.delete_entity_cli = self.create_client(DeleteEntity, '/delete_entity')

        self.remaining_balls = [f'{BALL_NAME_PREFIX}{i + 1}' for i in range(NUM_BALLS)]
        self.touched_count = 0
        self.check_in_progress = False

        self.get_logger().info(f'開始監看 front_chassis 觸碰，共 {NUM_BALLS} 顆球。可以手動開車測試了。')
        self.timer = self.create_timer(CHECK_PERIOD_SEC, self._check)

    def _check(self):
        if not self.remaining_balls or self.check_in_progress:
            return
        if not self.get_entity_cli.service_is_ready():
            return
        self.check_in_progress = True
        req = GetEntityState.Request()
        req.name = 'tennis_bot::base_link'
        future = self.get_entity_cli.call_async(req)
        future.add_done_callback(self._on_robot_pose)

    def _on_robot_pose(self, future):
        try:
            resp = future.result()
        except Exception:
            self.check_in_progress = False
            return
        if resp is None or not resp.success:
            self.check_in_progress = False
            return
        p = resp.state.pose.position
        o = resp.state.pose.orientation
        robot_pose = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))

        balls_to_check = list(self.remaining_balls)
        self._pending_ball_replies = len(balls_to_check)
        if self._pending_ball_replies == 0:
            self.check_in_progress = False
            return
        for name in balls_to_check:
            req = GetEntityState.Request()
            req.name = name
            future = self.get_entity_cli.call_async(req)
            future.add_done_callback(lambda f, n=name, rp=robot_pose: self._on_ball_pose(f, n, rp))

    def _on_ball_pose(self, future, name, robot_pose):
        self._pending_ball_replies -= 1
        if self._pending_ball_replies <= 0:
            self.check_in_progress = False

        try:
            resp = future.result()
        except Exception:
            return
        if resp is None or not resp.success or name not in self.remaining_balls:
            return

        (rx, ry, rz), (qx, qy, qz, qw) = robot_pose
        p = resp.state.pose.position
        dx, dy, dz = p.x - rx, p.y - ry, p.z - rz
        lx, ly, lz = quat_rotate_inverse(qx, qy, qz, qw, dx, dy, dz)

        in_front = (FRONT_TOUCH_X_RANGE[0] <= lx <= FRONT_TOUCH_X_RANGE[1]
                    and FRONT_TOUCH_Y_RANGE[0] <= ly <= FRONT_TOUCH_Y_RANGE[1])
        if in_front and name in self.remaining_balls:
            self.remaining_balls.remove(name)
            self.touched_count += 1
            self.get_logger().info(
                f'前車體碰到 {name}！({self.touched_count}/{NUM_BALLS}) '
                f'局部座標=({lx:.2f}, {ly:.2f})'
            )
            del_req = DeleteEntity.Request()
            del_req.name = name
            self.delete_entity_cli.call_async(del_req)
            if not self.remaining_balls:
                self.get_logger().info('全部球都碰過了！')


def main(args=None):
    rclpy.init(args=args)
    node = BallToucher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
