"""C4b 角落測試 (新車 v2)：C4 整場跑時 15° 三趟都在「底線靠邊線的角落」出界 —— 碰底線 EDGE 轉 ~113° 後車頭對著邊線，
黃線一直留在畫面裡 (EDGE 不斷)，D 的 EdgeUartGate 要連續 SAFE 0.5 s 才重新允許送 EDGE，所以第二條線沒有 EDGE，車子直接開出去。
這裡專門測這個情況：車子垂直 (或斜 45° 朝角落) 開向底線，離邊線 d_side 公尺，看轉完之後會不會越過任何一條黃線。

Gazebo 要先開著 (V2_CAM_PITCH_DEG=<仰角> WORLD=indoor_court.world bash experiments/v2_restart.sh)。
用法: python3 experiments/camera_tilt/corner_test.py <out_dir> --tilt 15 [--corners L,R] [--sides 0.6,1.2] [--angles 90,45] [--seeds 1,2]
輸出 <out_dir>/corner_summary.csv：front_out = 車頭最遠超出黃線框多少 m (正 = 越線)、n_edge = 這趟收到幾次 EDGE
"""
import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from edge_stop_test import BALLS, FOOTPRINT, NOSE, Driver, start, stop  # noqa: E402

X0, X1, Y = -11.885 + 0.20, -1.0, 5.485 - 0.20     # half_court 黃毛線框 (YARN_X0, YARN_X1, ±YARN_Y)
PATH_LEN = 3.0
NEUTRAL = (-6.3, 0.0)        # 半場中間
FRONT = [(0.248, 0.12), (0.248, -0.12)]


def out_dist(x, y, th, pts):
    c, s = math.cos(th), math.sin(th)
    m = -9.0
    for px, py in pts:
        wx, wy = x + px * c - py * s, y + px * s + py * c
        m = max(m, wx - X1, X0 - wx, wy - Y, -Y - wy)
    return m


def run_trial(n, out, tilt, corner, d_side, angle, seed):
    # 朝 -x 開向底線 (x = X0)；L 角落 = y 接近 +Y，R = y 接近 -Y；angle 90 = 垂直底線，45 = 往角落斜 45°
    side = 1.0 if corner == 'L' else -1.0
    yaw = math.pi - side * math.radians(90 - angle)
    hit_x, hit_y = X0 + 0.007, side * (Y - d_side)
    sx = hit_x - (PATH_LEN + NOSE[0]) * math.cos(yaw)
    sy = hit_y - (PATH_LEN + NOSE[0]) * math.sin(yaw)
    tag = f'tilt{tilt:g}_{corner}{d_side:g}_ang{angle}_s{seed}'
    tdir = out / tag
    tdir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, TENNISBOT_ROBOT='v2', TENNISBOT_JETSON_SEED=str(seed),
               TENNISBOT_JETSON_LOG_DIR=str(tdir), TENNISBOT_STM32_LOG_DIR=str(tdir), TEST_NAME='sim')
    # 開機在場地中間 (離每條黃線 >= 4 m，D 的閘門是 SAFE 狀態)，STM32 開始巡邏之後再把車移到起點。
    # (第一版直接在起點開機：低仰角時起點旁邊的邊線一開機就是 EDGE，那次 EDGE 在 STM32 還沒啟動前就被閘門用掉，
    #  之後黃線一直在畫面裡、閘門不再放行 -> 測試本身造成的越線，2026-10-26 12° 的結果因此作廢)
    n.stop_car()
    n.setp('tennis_bot', NEUTRAL[0], NEUTRAL[1], 0.01, yaw)
    n.uart_seen = 0
    jet = start(['ros2', 'run', 'tennis_bot', 'jetson_d_node'], env, tdir / 'jetson.log')
    t0 = time.time()
    while n.uart_seen == 0 and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.stop_car()
    n.setp('tennis_bot', NEUTRAL[0], NEUTRAL[1], 0.01, yaw)
    n.events = []
    stm = start(['ros2', 'run', 'tennis_bot', 'stm32_b_node'], env, tdir / 'stm32.log')
    t0 = time.time()
    booted = False
    while time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.05)
        bx, by, _ = n.pose()
        if math.hypot(bx - NEUTRAL[0], by - NEUTRAL[1]) > 0.3:
            booted = True
            break
    if booted:
        n.spin_for(1.0)                  # 巡邏 1 s：閘門確定是 SAFE (>= 0.5 s)
    n.events = []
    n.setp('tennis_bot', sx, sy, 0.01, yaw)
    rows, edges, turns = [], [], []
    t_start = n.sim_t
    last_done = None
    front_max = body_max = -9.0
    moved_t = None
    while True:
        rclpy.spin_once(n, timeout_sec=0.0)
        x, y, th = n.pose()
        st = n.sim_t
        if moved_t is None and math.hypot(x - sx, y - sy) > 0.05:
            moved_t = st
        f, b = out_dist(x, y, th, FRONT), out_dist(x, y, th, FOOTPRINT)
        front_max, body_max = max(front_max, f), max(body_max, b)
        rows.append((f'{st:.3f}', f'{x:.4f}', f'{y:.4f}', f'{math.degrees(th):.2f}', f'{f:.4f}', f'{b:.4f}'))
        for e in n.events:
            _, name, detail = (e.split(',', 2) + ['', ''])[:3]
            if name == 'EDGE':
                edges.append((st, detail))
            if name == 'TURN_START':
                turns.append([st, None, None])
                turns[-1][2] = th
            if name == 'TURN_DONE' and turns:
                turns[-1][1] = th
                last_done = st
        n.events = []
        if f > 0.8:                       # 已經開出去了
            break
        if last_done is not None and st - last_done > 6.0:
            break
        if st - t_start > 45:
            break
        time.sleep(0.02)
    stop(stm)
    stop(jet)
    n.stop_car()
    with open(tdir / 'pose.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['sim_t', 'x', 'y', 'yaw_deg', 'front_out', 'body_out'])
        w.writerows(rows)
    real_turns = [round(math.degrees((t1 - t0 + 3 * math.pi) % (2 * math.pi) - math.pi)) for _, t1, t0 in
                  [(a, b, c) for a, b, c in turns if b is not None]]
    res = 'CROSSED' if front_max > 0 else 'OK'
    if not booted or moved_t is None or moved_t - t_start > 1.0 or (real_turns and all(abs(t) < 60 for t in real_turns)):
        res = 'INVALID_NO_RESPONSE'
    return dict(tag=tag, tilt=tilt, corner=corner, d_side=d_side, angle=angle, seed=seed, result=res,
                n_edge=len(edges), edges=' | '.join(f'{t - t_start:.1f}s {d}' for t, d in edges),
                real_turns=' '.join(str(t) for t in real_turns), front_out=front_max, body_out=body_max)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--tilt', type=float, required=True)
    ap.add_argument('--corners', default='L,R')
    ap.add_argument('--sides', default='0.6,1.2,2.0')
    ap.add_argument('--angles', default='90,60,45')
    ap.add_argument('--seeds', default='1')
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Driver()
    for i, b in enumerate(BALLS):
        n.setp(b, 30 + 0.3 * i, 30, 0.033)
    summ = out / 'corner_summary.csv'
    new = not summ.exists()
    keys = ['tag', 'tilt', 'corner', 'd_side', 'angle', 'seed', 'result', 'n_edge', 'edges', 'real_turns',
            'front_out', 'body_out']
    with open(summ, 'a', newline='') as fh:
        w = csv.DictWriter(fh, keys)
        if new:
            w.writeheader()
        for c in a.corners.split(','):
            for ds in [float(v) for v in a.sides.split(',')]:
                for ang in [int(v) for v in a.angles.split(',')]:
                    for seed in [int(v) for v in a.seeds.split(',')]:
                        r = run_trial(n, out, a.tilt, c, ds, ang, seed)
                        w.writerow(r)
                        fh.flush()
                        print(f"{r['tag']}: {r['result']:8s} EDGE {r['n_edge']} 次 [{r['edges']}] 實際轉 {r['real_turns']} "
                              f"車頭最遠超出 {r['front_out']:+.2f} m", flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
