"""網球滾動阻力校正：把 ball_1 放到空地上，給定初速 (純滾動，角速度 = v/r)，
用 Gazebo 真實座標記錄它滾多遠、多久停下來。

用法 (Gazebo 要先開著，世界裡要有 ball_1)：
  python3 experiments/ball_roll_test.py "0.3,1.0"
輸出每個初速：10 秒內滾的距離、最終距離、速度掉到 0.01 m/s 以下的時間。
真實網球參考：μr≈0.02 庫倫型滾動阻力 -> 0.3 m/s 滾 ~0.38 m、1.0 m/s 滾 ~4.2 m。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState, SetEntityState

BALL = 'ball_1'
R = 0.033
START = (-8.0, 3.0)  # 我方半場空地 (網子在 x=0)，球往 +x 滾
WATCH_SEC = 15.0


class RollTest(Node):
    def __init__(self):
        super().__init__('ball_roll_test')
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_cli.wait_for_service()
        self.set_cli.wait_for_service()

    def call(self, cli, req):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return f.result()

    def pos(self):
        req = GetEntityState.Request()
        req.name = BALL
        r = self.call(self.get_cli, req)
        return r.state.pose.position.x, r.state.pose.position.y

    def launch(self, v):
        req = SetEntityState.Request()
        req.state.name = BALL
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = START[0], START[1], R
        req.state.pose.orientation.w = 1.0
        req.state.twist.linear.x = v
        req.state.twist.angular.y = v / R  # 往 +x 純滾動
        req.state.reference_frame = 'world'
        self.call(self.set_cli, req)

    def run(self, v):
        # 先讓球靜止落地
        self.launch(0.0)
        time.sleep(1.0)
        self.launch(v)
        t0 = time.time()
        last = self.pos()
        last_t = t0
        d10, stop_t = None, None
        while time.time() - t0 < WATCH_SEC:
            time.sleep(0.1)
            p = self.pos()
            now = time.time()
            speed = math.hypot(p[0] - last[0], p[1] - last[1]) / (now - last_t)
            if d10 is None and now - t0 >= 10.0:
                d10 = math.hypot(p[0] - START[0], p[1] - START[1])
            if stop_t is None and now - t0 > 0.5 and speed < 0.01:
                stop_t = now - t0
            last, last_t = p, now
        d_final = math.hypot(last[0] - START[0], last[1] - START[1])
        print(f'v0={v:.2f} m/s: 10s 內 {d10:.2f} m，{WATCH_SEC:.0f}s 時 {d_final:.2f} m，'
              f'停下 (<0.01 m/s) 時間 {stop_t if stop_t is not None else ">%.0f" % WATCH_SEC} s', flush=True)


def main():
    speeds = [float(x) for x in sys.argv[1].split(',')] if len(sys.argv) > 1 else [0.3, 1.0]
    rclpy.init()
    node = RollTest()
    for v in speeds:
        node.run(v)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
