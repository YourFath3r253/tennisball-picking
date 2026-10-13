"""實體車模式的「裁判」節點：自己不開車，只負責模擬才需要的事。

車子由 jetson_d_node (D 同學程式) + stm32_b_node (B 同學 main.c) 控制，跟實體車同一套邏輯。這個節點：
  - 把車傳送到起點 (半場右後角，朝網子)、滾輪常轉 (實體車滾輪 PWM 950 一直轉)
  - 用 Gazebo 真實座標判斷撿到幾顆球 (球進後車廂)、記錄軌跡 (模擬時間戳，可以跟 Jetson / STM32 的 CSV 對齊)
  - 結束條件：撿滿 / 逾時 (模擬時間) / 出界 (離黃毛線超過 1.5 m，保護用)
沿用 BoundaryBounceNode (GridPatrolNode) 的撿球判定與紀錄，覆寫控制迴圈為「只看不動」。

環境變數：TENNISBOT_BOUNCE_TIMEOUT (逾時秒數，模擬時間，預設 900)
"""
import csv
import math

import rclpy

from tennis_bot import half_court
from tennis_bot.boundary_bounce_node import BoundaryBounceNode, BOUNCE_TIMEOUT_SEC
from tennis_bot.grid_patrol_node import NUM_BALLS

OUT_OF_YARN_MARGIN_M = 1.5


class RealCarNode(BoundaryBounceNode):
    def __init__(self):
        super().__init__()
        self.state = 'REAL_CAR'
        self.sim_traj_file = open(self.run_dir / 'real_traj.csv', 'w', newline='')
        self.sim_traj = csv.writer(self.sim_traj_file)
        self.sim_traj.writerow(['sim_t', 'real_x', 'real_y', 'real_yaw_deg'])
        with open(self.run_dir / 'meta.txt', 'a') as f:
            f.write('mode=real_car (jetson_d_node + stm32_b_node)\n')
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
