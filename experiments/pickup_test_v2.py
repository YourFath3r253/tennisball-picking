"""新車體 v2 撿球道驗證 (組裝前)：車子直線開向一顆球，滾輪常轉，記錄球在車體座標的軌跡、判斷最後停在哪。

Gazebo 要先開著：TENNISBOT_ROBOT=v2 WORLD=indoor_court.world bash experiments/sim_up.sh
用法: python3 experiments/pickup_test_v2.py <out.csv> [--W 42.5] [--v 0.41] [--offsets 0,0.02,-0.02] [--second 0.25]
  W        滾輪指令 (roller_cmd angular.z；每顆滾輪 W*0.07/0.045 rad/s，舊車 42.5 = 66 rad/s)
  v        車速 m/s (實體車追球 Forward 75 rpm = 0.41)
  offsets  球偏離中線多少 m (左正)
  second   >0：同一趟在第一顆後面再放第二顆 (間隔 m)，測「下一顆把卡住的推進車廂」
球的質量看 world 檔 (gen_ball_layout.py --mass)。

結果 (每顆球)：basket 進後車廂 / deck 停在電子艙頂板 / stuck 卡在機構裡 / ejected 被甩出車外 /
missed 沒撿起來 (車子開過去或推著走)
"""
import argparse
import csv
import math
import time

import rclpy
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node

X0, Y0 = -8.0, 0.0          # 車子起點 (場地中間空地)
BALL_AHEAD = 0.60           # 球放在 base_link 前方多遠
DRIVE_DIST = 1.30           # 車子總共開多遠 (m)，之後停車再觀察 3 s
BASKET_X, BASKET_Y, BASKET_ZMAX = (-0.51, -0.31), 0.19, 0.15
ROBOT_X, ROBOT_Y = (-0.52, 0.26), 0.21   # 車體外框 (base_link 座標)
BOX_X = (-0.30, -0.10)                   # 電子艙箱子內部 (頂板 z 0.14 以下)


def quat_yaw(o):
    return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))


class Test(Node):
    def __init__(self):
        super().__init__('pickup_test_v2')
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_cli.wait_for_service()
        self.set_cli.wait_for_service()
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.roller = self.create_publisher(Twist, '/roller_cmd', 10)

    def call(self, cli, req):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return f.result()

    def setp(self, name, x, y, z, yaw=0.0):
        r = SetEntityState.Request()
        r.state.name = name
        r.state.pose.position.x, r.state.pose.position.y, r.state.pose.position.z = float(x), float(y), float(z)
        r.state.pose.orientation.z, r.state.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        r.state.reference_frame = 'world'
        self.call(self.set_cli, r)

    def pose(self, name):
        r = GetEntityState.Request()
        r.name = name
        s = self.call(self.get_cli, r).state
        return s.pose.position, s.pose.orientation, s.twist.linear

    def twist(self, v=0.0, w_roller=None):
        t = Twist()
        t.linear.x = float(v)
        self.cmd.publish(t)
        if w_roller is not None:
            r = Twist()
            r.angular.z = float(w_roller)
            self.roller.publish(r)


def classify(trace):
    """trace: [(t, lx, ly, lz, speed)] 車體座標。看最後 1 s。"""
    tail = [p for p in trace if p[0] >= trace[-1][0] - 1.0]
    lx = sum(p[1] for p in tail) / len(tail)
    ly = sum(p[2] for p in tail) / len(tail)
    lz = sum(p[3] for p in tail) / len(tail)
    zmax = max(p[3] for p in trace)
    inside = ROBOT_X[0] <= lx <= ROBOT_X[1] and abs(ly) <= ROBOT_Y
    if BOX_X[0] < lx < BOX_X[1] and abs(ly) < 0.18 and lz < 0.125:
        res = 'TUNNEL'      # 停在電子艙箱子裡面 = 穿牆 (數值問題，不是設計問題)
    elif BASKET_X[0] <= lx <= BASKET_X[1] and abs(ly) <= BASKET_Y and lz <= BASKET_ZMAX:
        res = 'basket'
    elif inside and -0.31 < lx < 0.02 and lz > 0.13:
        res = 'deck'
    elif inside and zmax > 0.06:
        res = 'stuck'
    elif zmax > 0.08:
        res = 'ejected'
    else:
        res = 'missed'
    t_basket = next((p[0] for p in trace if BASKET_X[0] <= p[1] <= BASKET_X[1] and abs(p[2]) <= BASKET_Y
                     and p[3] <= BASKET_ZMAX), None)
    return res, lx, ly, lz, zmax, t_basket


def run_trial(n, W, v, off, second, writer, label):
    balls = ['ball_1'] + (['ball_2'] if second > 0 else [])
    for i, b in enumerate(['ball_%d' % k for k in range(1, 11)]):
        n.setp(b, 8 + 0.3 * i, 8, 0.033)          # 全部移開
    n.twist(0.0, W)
    n.setp('tennis_bot', X0, Y0, 0.01, 0.0)
    t0 = time.time()
    while time.time() - t0 < 1.5:
        n.twist(0.0, W)
        rclpy.spin_once(n, timeout_sec=0.05)
    n.setp('ball_1', X0 + BALL_AHEAD, Y0 + off, 0.0335)
    if second > 0:
        n.setp('ball_2', X0 + BALL_AHEAD + second, Y0 + off, 0.0335)
    traces = {b: [] for b in balls}
    p0, _, _ = n.pose('tennis_bot')
    start = time.time()
    stop_t = None
    while True:
        now = time.time() - start
        rp, ro, _ = n.pose('tennis_bot')
        dist = math.hypot(rp.x - p0.x, rp.y - p0.y)
        driving = dist < DRIVE_DIST and now < 15.0
        if not driving and stop_t is None:
            stop_t = now
        n.twist(v if driving else 0.0, W)
        yaw = quat_yaw(ro)
        for b in balls:
            bp, _, bv = n.pose(b)
            dx, dy = bp.x - rp.x, bp.y - rp.y
            lx = dx * math.cos(yaw) + dy * math.sin(yaw)
            ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
            sp = math.sqrt(bv.x ** 2 + bv.y ** 2 + bv.z ** 2)
            traces[b].append((now, lx, ly, bp.z, sp))
            writer.writerow([label, b, f'{now:.3f}', f'{lx:.4f}', f'{ly:.4f}', f'{bp.z:.4f}', f'{sp:.3f}'])
        rclpy.spin_once(n, timeout_sec=0.0)
        if stop_t is not None and now - stop_t > 3.0:
            break
        time.sleep(0.02)
    n.twist(0.0, W)
    return {b: classify(traces[b]) for b in balls}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--W', type=float, default=42.5)
    ap.add_argument('--v', type=float, default=0.41)
    ap.add_argument('--offsets', default='0')
    ap.add_argument('--second', type=float, default=0.0)
    ap.add_argument('--repeat', type=int, default=1)
    a = ap.parse_args()
    rclpy.init()
    n = Test()
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['trial', 'ball', 't', 'lx', 'ly', 'z', 'speed'])
        for off in [float(x) for x in a.offsets.split(',')]:
            for k in range(a.repeat):
                label = f'W{a.W}_v{a.v}_off{off:+.2f}_r{k}'
                res = run_trial(n, a.W, a.v, off, a.second, w, label)
                for b, (r, lx, ly, lz, zmax, tb) in res.items():
                    print(f'{label} {b}: {r:8s} 最後位置 (車體座標) x={lx:+.3f} y={ly:+.3f} z={lz:.3f} 最高 z={zmax:.3f}'
                          + (f' 進車廂 {tb:.2f}s' if tb is not None else ''), flush=True)
    n.twist(0.0, 0.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
