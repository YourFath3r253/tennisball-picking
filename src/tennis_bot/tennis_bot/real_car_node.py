"""實體車模式的「裁判」節點：自己不開車，只負責模擬才需要的事。

車子由 jetson_d_node (D 同學程式) + stm32_b_node (B 同學 main.c) 控制，跟實體車同一套邏輯。這個節點：
  - 把車傳送到起點 (半場右後角，朝網子)、滾輪常轉 (實體車滾輪 PWM 950 一直轉)
  - 用 Gazebo 真實座標判斷撿到幾顆球 (球進後車廂)、記錄軌跡 (模擬時間戳，可以跟 Jetson / STM32 的 CSV 對齊)
  - 結束條件：撿滿 / 逾時 (模擬時間) / 出界 (離黃毛線超過 1.5 m，保護用)
沿用 BoundaryBounceNode (GridPatrolNode) 的撿球判定與紀錄，覆寫控制迴圈為「只看不動」。

環境變數：TENNISBOT_BOUNCE_TIMEOUT (逾時秒數，模擬時間，預設 900)
         TENNISBOT_PICK_AT_MOUTH=1：球一進到撿球口 (新車 v2 前段兩側板之間) 就算撿到並移出場外，
           把「找球/追球/邊界」跟「滾輪能不能把球送進車廂」拆開測 (2026-10-26 鏡頭仰角研究用)
"""
import csv
import math
import os
import time

import rclpy
from gazebo_msgs.srv import SetEntityState

from tennis_bot import half_court
from tennis_bot.boundary_bounce_node import BoundaryBounceNode, BOUNCE_TIMEOUT_SEC
from tennis_bot.grid_patrol_node import NUM_BALLS, quat_rotate_inverse

OUT_OF_YARN_MARGIN_M = 1.5
PICK_AT_MOUTH = os.environ.get('TENNISBOT_PICK_AT_MOUTH', '') == '1'
# 撿球口範圍 (v2 base_link 座標)：車頭 x=0.248、滾輪中心 x≈0.10、前段兩側板內寬約 ±0.12；球心進到這裡 = 已經被吃進去
MOUTH_X = (-0.10, 0.27)
MOUTH_Y_HALF = 0.13
MOUTH_Z_MAX = 0.30


class RealCarNode(BoundaryBounceNode):
    def __init__(self):
        super().__init__()
        self.state = 'REAL_CAR'
        self.sim_traj_file = open(self.run_dir / 'real_traj.csv', 'w', newline='')
        self.sim_traj = csv.writer(self.sim_traj_file)
        self.sim_traj.writerow(['sim_t', 'real_x', 'real_y', 'real_yaw_deg'])
        with open(self.run_dir / 'meta.txt', 'a') as f:
            f.write('mode=real_car (jetson_d_node + stm32_b_node)\n')
            if PICK_AT_MOUTH:
                f.write('pick_at_mouth=1 (球進撿球口就算撿到並移走)\n')
        self.set_entity_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_logger().info(f'實體車模式：裁判節點 (不控制車子)，逾時 {BOUNCE_TIMEOUT_SEC:.0f}s，球 {NUM_BALLS} 顆')

    def _is_out_of_bounds(self, x, y):
        return not half_court.inside_yarn(x, y, OUT_OF_YARN_MARGIN_M)

    def _on_ball_seen(self, now):
        pass   # 不控制

    def _drive(self, v, w):
        pass   # 保險：任何繼承來的路徑都不能送輪速

    def _set_wheels(self, l, r):
        pass

    def _stop(self):
        pass

    def _control_loop(self):
        if not self._have_odom or self.done:
            return
        if not self.remaining_balls:
            self._finish_run('撿滿')
            return
        if self._sim_elapsed() > BOUNCE_TIMEOUT_SEC:
            self._finish_run('逾時')
            return
        self._check_touch()

    def _on_ball_pose(self, future, name, robot_pose):
        if not PICK_AT_MOUTH:
            super()._on_ball_pose(future, name, robot_pose)
            return
        self.touch_check_in_progress = False
        try:
            resp = future.result()
        except Exception:
            return
        if resp is None or not resp.success or name not in self.remaining_balls:
            return
        p = resp.state.pose.position
        (rx, ry, rz), (qx, qy, qz, qw) = robot_pose
        lx, ly, lz = quat_rotate_inverse(qx, qy, qz, qw, p.x - rx, p.y - ry, p.z - rz)
        if not (MOUTH_X[0] <= lx <= MOUTH_X[1] and abs(ly) <= MOUTH_Y_HALF and lz <= MOUTH_Z_MAX):
            return
        self.remaining_balls.remove(name)
        self.touched_count += 1
        elapsed = time.time() - self.start_time
        self.get_logger().info(f'{name} 進撿球口 (算撿到並移走) ({self.touched_count}/{NUM_BALLS}) '
                               f'位置=({lx:.2f}, {ly:.2f}, {lz:.2f})')
        if not self.done:
            self.touch_writer.writerow([name, f'{elapsed:.2f}', self.wp_index, self._current_cell_number(),
                                        f'{self._sim_elapsed():.2f}'])
            self.touch_file.flush()
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = \
            30.0 + 0.3 * self.touched_count, 30.0, 0.033
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self.set_entity_cli.call_async(req)

    def _log_trajectory(self):
        super()._log_trajectory()
        if self.done or self.real_x is None or self._last_stamp is None:
            return
        self.sim_traj.writerow([f'{self._last_stamp:.3f}', f'{self.real_x:.3f}', f'{self.real_y:.3f}',
                                f'{math.degrees(self.real_yaw):.2f}'])
        self.sim_traj_file.flush()

    def _finish_run(self, reason):
        was_done = self.done
        super()._finish_run(reason)
        if not was_done:
            self.sim_traj_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = RealCarNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
