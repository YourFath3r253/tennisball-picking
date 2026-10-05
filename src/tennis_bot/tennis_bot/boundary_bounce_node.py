"""D 同學的路徑規劃提案：半場、四色邊界、碰到邊界轉固定角度 (沒有弓字路徑、不需要里程計)。

  CRUISE      直走 (陀螺儀維持航向，0.4 m/s)，一路走到相機看到邊界色帶 (0.6 m 內)
  TURN        原地轉 BOUNCE_ANGLE_DEG 度 (陀螺儀量角度)，轉完繼續 CRUISE
  ALIGN / APPROACH / BLIND_DASH
              跟弓字版完全一樣 (直接沿用 GridPatrolNode)：看到球 -> 原地對準 -> 邊走邊修正 ->
              看不到球就盲衝；撿完以「當下的方向」繼續 CRUISE (每撿一顆球方向就換一次)

轉向方向：邊界色帶在畫面左半邊比較多 -> 往右轉 (遠離邊界)，右半邊比較多 -> 往左轉，
正對 (CENTER) -> 固定往左轉。長邊、短邊、網前線都用同一套規則 (Sean 指定)。

追球途中只有「網前線 (CYAN)」會中斷追球 (網子是實體障礙)，其他三條線不中斷 (球都在線內，
盲衝結束後回到 CRUISE 再判斷要不要轉)。

結束條件：撿滿 / 模擬時間超過 BOUNCE_TIMEOUT_SEC (逾時) / 真實座標離四條線超過 1.5 m (出界)。
沒有里程計也能跑 (轉角度只用陀螺儀)；里程計還是照算，只拿來畫圖對照。

環境變數：
  TENNISBOT_BOUNCE_ANGLE_DEG  轉向角度 (預設 135)
  TENNISBOT_BOUNCE_TIMEOUT    逾時秒數 (模擬時間，預設 900)
"""
import csv
import math
import os
import time

import rclpy
from std_msgs.msg import Empty

from tennis_bot import half_court
from tennis_bot.grid_patrol_node import (
    GridPatrolNode, angle_diff, PATROL_SPEED, PATROL_MAX_OMEGA, NUM_BALLS)

BOUNCE_ANGLE_DEG = float(os.environ.get('TENNISBOT_BOUNCE_ANGLE_DEG', '135'))
BOUNCE_TIMEOUT_SEC = float(os.environ.get('TENNISBOT_BOUNCE_TIMEOUT', '900'))
TURN_KP = 1.5
TURN_MAX_OMEGA = 0.6
TURN_MIN_OMEGA = 0.15        # 快轉到時不要越轉越慢爬不到
TURN_DONE_DEG = 3.0
CRUISE_HEADING_KP = 1.2
NET_LINE_COLOR = 'CYAN'
OUT_OF_LINES_MARGIN_M = 1.5  # 真實座標離四條線超過這個距離就算出界 (保護用)
# 轉完之後多久內不理 COURT EDGE：UART 字串沒有拍攝時間，轉完後先收到的幾筆其實是轉向途中拍的舊畫面
# (畫面延遲 0.60 s)，會誤判成「又碰到邊界」再轉一次。實體車 STM32 也拿不到拍攝時間，只能這樣等。
COURT_SETTLE_SEC = 0.7


class BoundaryBounceNode(GridPatrolNode):
    def __init__(self):
        super().__init__()
        self.state = 'CRUISE'
        self.cruise_heading = self.yaw
        self._cruise_since = 0.0
        self.turn_target = None
        self.bounce_count = 0

        self.bounce_file = open(self.run_dir / 'bounces.csv', 'w', newline='')
        self.bounce_writer = csv.writer(self.bounce_file)
        self.bounce_writer.writerow(['t', 'sim_t', 'color', 'side', 'turn_deg', 'during_state',
                                     'real_x', 'real_y', 'yaw_deg'])
        with open(self.run_dir / 'meta.txt', 'a') as f:
            f.write(f'mode=boundary_bounce\nbounce_angle_deg={BOUNCE_ANGLE_DEG}\n'
                    f'timeout_sec={BOUNCE_TIMEOUT_SEC}\n')
        self.get_logger().info(
            f'D 同學提案：半場四色邊界，碰到邊界轉 {BOUNCE_ANGLE_DEG:.0f}°，逾時 {BOUNCE_TIMEOUT_SEC:.0f}s (模擬時間)')

    # ---- 起點、出界範圍換成半場 ----
    def _start_pose(self):
        return half_court.START_POSE

    def _is_out_of_bounds(self, x, y):
        return not half_court.inside_lines(x, y, OUT_OF_LINES_MARGIN_M)

    def _heading_into_net(self):
        return False  # 不用里程計判斷網子，改由網前色帶 (CYAN) 處理

    # ---- 感測 (COURT / BALL 都從 /jetson_uart 來，見 GridPatrolNode._uart_cb) ----
    def _fresh_edge(self):
        """最新一筆 COURT 是 EDGE，而且是這段 CRUISE 開始 COURT_SETTLE_SEC 之後才收到的。"""
        c = self._court
        return c if c is not None and c[0] == 'EDGE' and c[4] >= self._cruise_since + COURT_SETTLE_SEC else None

    def _on_ball_seen(self, now):
        if self._vision_ignore_until is not None and now < self._vision_ignore_until:
            return
        if self.state in ('CRUISE', 'TURN'):
            self.get_logger().info(f'發現網球 -> 對準 (猜是{self._estimate_target_ball() or "?"}，原本在 {self.state})')
            self.state = 'ALIGN'
            self.chase_cmd_log = []

    # ---- 控制迴圈 ----
    def _control_loop(self):
        if not self._have_odom or self.done:
            return
        if not self.remaining_balls:
            self._finish_run('撿滿')
            return
        if self._sim_elapsed() > BOUNCE_TIMEOUT_SEC:
            self._finish_run('逾時')
            return

        now = self.get_clock().now().nanoseconds / 1e9
        self._check_touch()

        edge = self._fresh_edge()
        if edge is not None and edge[3] == NET_LINE_COLOR and self.state in ('ALIGN', 'APPROACH', 'BLIND_DASH'):
            self.get_logger().info('追球途中看到網前線 -> 放棄這顆球，轉向')
            self._vision_ignore_until = now + 3.0
            self.vision_reset_pub.publish(Empty())
            self._begin_turn(edge)
            return

        if self.state == 'CRUISE':
            self._do_cruise()
        elif self.state == 'TURN':
            self._do_turn()
        elif self.state == 'ALIGN':
            self._do_align(now)
        elif self.state == 'APPROACH':
            self._do_approach(now)
        elif self.state == 'BLIND_DASH':
            self._do_blind_dash(now)

    def _start_return(self):
        # 追球結束 (撿到 / 看丟 / 盲衝完)：用「當下的方向」繼續直走
        self.chase_cmd_log = []
        self._enter_cruise(self.yaw)

    def _enter_cruise(self, heading):
        self.state = 'CRUISE'
        self.cruise_heading = heading
        self._cruise_since = self.get_clock().now().nanoseconds / 1e9

    def _do_cruise(self):
        edge = self._fresh_edge()
        if edge is not None:
            self._begin_turn(edge)
            return
        err = angle_diff(self.cruise_heading, self.yaw)
        omega = max(-PATROL_MAX_OMEGA, min(PATROL_MAX_OMEGA, CRUISE_HEADING_KP * err))
        self._drive(PATROL_SPEED * max(0.0, 1.0 - abs(err) / (math.pi / 2)), omega)

    def _begin_turn(self, edge):
        _, side, _, color, _ = edge
        # 邊界在左 -> 右轉 (順時針, 負)；邊界在右或正前方 -> 左轉 (逆時針, 正)
        sign = -1.0 if side == 'LEFT' else 1.0
        turn = math.radians(BOUNCE_ANGLE_DEG) * sign
        self.turn_target = math.atan2(math.sin(self.yaw + turn), math.cos(self.yaw + turn))
        self._turn_sign = sign
        prev_state = self.state
        self.state = 'TURN'
        self.bounce_count += 1
        self._stop()
        self.get_logger().info(
            f'邊界 {color}({half_court.LINES.get(color, {}).get("label", "?")}) 在 {side} -> '
            f'轉 {math.degrees(turn):+.0f}° (第 {self.bounce_count} 次)')
        if not self.done:
            self.bounce_writer.writerow([
                f'{time.time() - self.start_time:.2f}', f'{self._sim_elapsed():.2f}', color, side,
                f'{math.degrees(turn):.0f}', prev_state,
                '' if self.real_x is None else f'{self.real_x:.3f}',
                '' if self.real_y is None else f'{self.real_y:.3f}', f'{math.degrees(self.yaw):.1f}'])
            self.bounce_file.flush()

    def _do_turn(self):
        err = angle_diff(self.turn_target, self.yaw)
        # 180° 時 angle_diff 正負號會在 ±π 附近跳，剩很多角度時強制照一開始決定的方向轉
        if abs(err) > math.radians(150):
            err = abs(err) * self._turn_sign
        if abs(err) < math.radians(TURN_DONE_DEG):
            self._stop()
            self._enter_cruise(self.turn_target)
            return
        omega = TURN_KP * err
        omega = math.copysign(max(TURN_MIN_OMEGA, min(TURN_MAX_OMEGA, abs(omega))), omega)
        self._drive(0.0, omega)

    def _finish_run(self, reason):
        was_done = self.done
        super()._finish_run(reason)
        if was_done:
            return
        self.bounce_file.close()
        with open(self.run_dir / 'result.txt', 'a') as f:
            f.write(f'bounce_count={self.bounce_count}\nbounce_angle_deg={BOUNCE_ANGLE_DEG}\n')
        self.get_logger().info(f'邊界轉向 {self.bounce_count} 次，撿到 {self.touched_count}/{NUM_BALLS}')


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryBounceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
