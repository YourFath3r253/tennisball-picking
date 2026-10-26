"""C2 邊界停車測試 (新車 v2)：車子用巡邏速度直直開向網前那條黃毛線，跑 D 的 Jetson 程式 (SIL) + B 的 STM32 邏輯，
量 EDGE 觸發、煞停、轉向過程中「車體最前面離線多遠」。

Gazebo 要先開著 (V2_CAM_PITCH_DEG=<仰角> WORLD=indoor_court.world bash experiments/v2_restart.sh)。
每一趟都重新開 jetson_d_node + stm32_b_node (像實體車重新開機)：先開 Jetson 等它送出第一行 UART，再把車放回起點、開 STM32。

用法: python3 experiments/camera_tilt/edge_stop_test.py <out_dir> --tilt 21.3 [--angles 90,60,-60,30,-30] [--seeds 1,2]
  angle = 車頭方向跟黃線的夾角 (90 = 垂直撞線；正 = 從右下往左上斜切，負 = 反方向)
環境變數照傳給兩個節點 (例如 TENNISBOT_PATROL_RPM=120)。

輸出：<out_dir>/summary.csv (每趟一列) + 每趟 trial_*.csv (車子真實位置，模擬時間)
  margin = 線的近側邊緣 - 車體輪廓最前點 (公尺；負 = 車體有一部分壓過/越過線)
  front_*：轉向開始前 (直線開過去 + 煞車) 的車頭；仰角主要影響這個
  turn_min_margin：原地轉向時整個輪廓 (新車驅動軸在前、後車廂長 0.51 m，轉大角度時車尾會甩過線)
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

LINE_X = -1.0                 # half_court.YARN_X1 (網前 1 m 那條黃毛線中心)
LINE_NEAR = LINE_X - 0.007    # 毛線寬 1.4 cm，車子從 x 小的那側過來
PATH_LEN = 4.0                # 起點沿車頭方向到線的距離 (m)，120 rpm 約 6 s，先到穩定巡邏速度
# 新車 v2 車體 + 驅動輪的俯視輪廓 (base_link 座標，從 meshes/v2 的凸包簡化；前緣 x=0.248)
FOOTPRINT = [(0.248, 0.12), (0.222, 0.149), (0.052, 0.183), (-0.101, 0.2), (-0.433, 0.2), (-0.508, 0.125),
             (-0.508, -0.125), (-0.433, -0.2), (-0.101, -0.2), (0.052, -0.183), (0.222, -0.149), (0.248, -0.12)]
NOSE = (0.248, 0.0)
BALLS = ['ball_%d' % i for i in range(1, 11)]


def front_x(x, y, yaw, pts):
    c, s = math.cos(yaw), math.sin(yaw)
    return max(x + px * c - py * s for px, py in pts)


class Driver(Node):
    def __init__(self):
        super().__init__('edge_stop_test')
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_cli.wait_for_service()
        self.set_cli.wait_for_service()
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sim_t = 0.0
        self.uart_seen = 0
        self.events = []
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'sim_t', m.clock.sec + m.clock.nanosec * 1e-9),
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/jetson_uart', self._uart, 50)
        self.create_subscription(String, '/stm32_event', lambda m: self.events.append(m.data), 50)

    def _uart(self, m):
        self.uart_seen += 1

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

    def pose(self):
        r = GetEntityState.Request()
        r.name = 'tennis_bot'
        p = self.call(self.get_cli, r).state.pose
        o = p.orientation
        return p.position.x, p.position.y, math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))

    def stop_car(self):
        for _ in range(5):
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_for(self, sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.02)


def start(cmd, env, log):
    return subprocess.Popen(cmd, env=env, stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)


def stop(p):
    if p is None or p.poll() is not None:
        return
    os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.wait()


def run_trial(n, out, tilt, angle, seed):
    yaw = math.copysign(math.radians(90.0 - abs(angle)), angle)    # 90 -> yaw 0 (朝 +x 垂直撞線)；60 -> +30°；-60 -> -30°
    # 起點：車頭 (NOSE) 沿車頭方向走 PATH_LEN 剛好碰到線中間 (y=0)
    hit_x, hit_y = LINE_NEAR, 0.0
    sx = hit_x - (PATH_LEN + NOSE[0]) * math.cos(yaw)
    sy = hit_y - (PATH_LEN + NOSE[0]) * math.sin(yaw)
    tag = f'tilt{tilt:g}_ang{angle:+d}_s{seed}'
    tdir = out / tag
    tdir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, TENNISBOT_ROBOT='v2', TENNISBOT_JETSON_SEED=str(seed),
               TENNISBOT_JETSON_LOG_DIR=str(tdir), TENNISBOT_STM32_LOG_DIR=str(tdir), TEST_NAME='sim')
    n.stop_car()
    n.setp('tennis_bot', sx, sy, 0.01, yaw)
    n.uart_seen = 0
    jet = start(['ros2', 'run', 'tennis_bot', 'jetson_d_node'], env, tdir / 'jetson.log')
    t0 = time.time()
    while n.uart_seen == 0 and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.uart_seen == 0:
        stop(jet)
        return dict(tag=tag, tilt=tilt, angle=angle, seed=seed, result='JETSON_NO_OUTPUT')
    n.stop_car()
    n.setp('tennis_bot', sx, sy, 0.01, yaw)
    n.events = []
    stm = start(['ros2', 'run', 'tennis_bot', 'stm32_b_node'], env, tdir / 'stm32.log')
    rows = []
    t_start = n.sim_t
    t_done = None
    min_margin = 1e9
    front_min = 1e9
    margin_edge = margin_stop = None
    edge_t = turn_t = None
    edge_detail = ''
    while True:
        rclpy.spin_once(n, timeout_sec=0.0)
        x, y, th = n.pose()
        st = n.sim_t
        m_body = LINE_NEAR - front_x(x, y, th, FOOTPRINT)
        m_nose = LINE_NEAR - front_x(x, y, th, [NOSE])
        min_margin = min(min_margin, m_body)
        if turn_t is None:
            front_min = min(front_min, m_body)
        rows.append((f'{st:.3f}', f'{x:.4f}', f'{y:.4f}', f'{math.degrees(th):.2f}', f'{m_body:.4f}', f'{m_nose:.4f}'))
        for e in n.events:
            _, name, detail = (e.split(',', 2) + ['', ''])[:3]
            if name == 'EDGE' and edge_t is None:
                edge_t, edge_detail, margin_edge = st, detail, m_body
            if name == 'TURN_START' and turn_t is None:
                turn_t, margin_stop = st, m_body
            if name == 'TURN_DONE' and t_done is None:
                t_done = st
        n.events = []
        if t_done is not None and st - t_done > 2.0:
            break
        if st - t_start > PATH_LEN / 0.3 + 15 or m_body < -1.5:
            break
        time.sleep(0.02)
    stop(stm)
    stop(jet)
    n.stop_car()
    with open(tdir / 'pose.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['sim_t', 'x', 'y', 'yaw_deg', 'margin_body', 'margin_nose'])
        w.writerows(rows)
    res = 'STOP_BEFORE' if front_min >= 0 else 'NOSE_OVER'
    # 有效性：B 的 TurnPrecise 期間車身真的有轉 (2026-10-26 遇過 stm32 節點的 /cmd_vel 晚 ~8 s 才接上，
    # 節點內部以為轉完了、車子其實沒動 -> 這趟作廢重跑)
    if turn_t is not None and t_done is not None:
        y0 = next(float(r[3]) for r in rows if float(r[0]) >= turn_t)
        y1 = next((float(r[3]) for r in rows if float(r[0]) >= t_done), float(rows[-1][3]))
        if abs((y1 - y0 + 540) % 360 - 180) < 60:
            res = 'INVALID_NO_RESPONSE'
    if edge_t is None:
        res = 'NO_EDGE_' + res
    elif t_done is None:
        res = 'NO_TURN_DONE_' + res
    return dict(tag=tag, tilt=tilt, angle=angle, seed=seed, result=res, edge_detail=edge_detail,
                margin_at_edge=margin_edge, margin_at_stop=margin_stop, front_min_margin=front_min,
                turn_min_margin=min_margin,
                t_edge=None if edge_t is None else edge_t - t_start)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--tilt', type=float, required=True)
    ap.add_argument('--angles', default='90,60,-60,30,-30')
    ap.add_argument('--seeds', default='1,2')
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Driver()
    for i, b in enumerate(BALLS):
        n.setp(b, 30 + 0.3 * i, 30, 0.033)    # 球全部移出場外
    summ = out / 'summary.csv'
    new = not summ.exists()
    keys = ['tag', 'tilt', 'angle', 'seed', 'result', 'edge_detail', 'margin_at_edge', 'margin_at_stop',
            'front_min_margin', 'turn_min_margin', 't_edge']
    with open(summ, 'a', newline='') as f:
        w = csv.DictWriter(f, keys)
        if new:
            w.writeheader()
        for ang in [int(v) for v in a.angles.split(',')]:
            for seed in [int(v) for v in a.seeds.split(',')]:
                r = run_trial(n, out, a.tilt, ang, seed)
                w.writerow(r)
                f.flush()
                fmt = lambda v: '--' if v is None else f'{v:+.3f}'
                print(f"{r['tag']}: {r['result']:14s} EDGE {r.get('edge_detail', '')!s:14s} "
                      f"邊緣距離 觸發時 {fmt(r.get('margin_at_edge'))} 停住時 {fmt(r.get('margin_at_stop'))} "
                      f"車頭最小 {fmt(r.get('front_min_margin'))} 轉向時車體最小 {fmt(r.get('turn_min_margin'))} m",
                      flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
