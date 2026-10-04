"""D 同學提案的「純幾何」快速模擬 (不開 Gazebo)：用來跑大量隨機佈局，補 Gazebo 每個角度只有 3 次的樣本數。

模型 (盡量跟 boundary_bounce_node + vision_node 一致，但忽略物理/打滑/視覺鎖定細節)：
- 半場四條線 (half_court.py)，車體用相機位置當一個點，直走 0.4 m/s、原地轉 0.6 rad/s
- 看到球：球在相機前方 3.0 m 內、水平視角 ±40° 以內 -> 原地轉向對準 -> 0.3 m/s 直走過去 ->
  撿到後再盲衝 0.25 m (盲衝結束點在球後面一點)，然後用「當下方向」繼續直走
- 看到邊界：畫面下方 ROI = 地面上前方 0.24~0.6 m、水平 ±40° 的梯形，梯形裡有任何一點超出邊界
  (= 色帶進到 ROI) -> 原地轉 x 度；色帶在左半邊比較多往右轉，否則往左轉 (跟 node 一樣)
- 逾時 900 s

用法: python3 experiments/bounce_2d_sim.py [n_layouts] [角度們]
  例: python3 experiments/bounce_2d_sim.py 200 "90 110 135 180"
輸出每個角度：全部撿完的比例、撿完時間中位數/平均、平均撿到幾顆；並畫出 seed 501 的軌跡比較圖。
"""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src' / 'tennis_bot' / 'tennis_bot'))
sys.path.insert(0, str(ROOT / 'experiments'))
import half_court as hc  # noqa: E402
from gen_ball_layout import random_positions_half_court  # noqa: E402

DT = 0.05
CRUISE_V = 0.4
APPROACH_V = 0.3
TURN_W = 0.6
DETECT_R = 3.0
HALF_FOV = math.radians(40)
ROI_NEAR, ROI_FAR = 0.24, 0.6
PICK_DIST = 0.05
BLIND_EXTRA = 0.25
TIMEOUT = 900.0
X0, X1 = hc.BASELINE_X, hc.NET_LINE_X
Y0, Y1 = -hc.SIDELINE_Y, hc.SIDELINE_Y

# ROI 梯形的取樣點 (車體座標：前方 d、左邊 l)
ROI_PTS = [(d, l) for d in [ROI_NEAR + i * (ROI_FAR - ROI_NEAR) / 6 for i in range(7)]
           for l in [d * math.tan(HALF_FOV) * (j / 6.0 - 1.0) for j in range(13)]]


def outside(x, y):
    return not (X0 <= x <= X1 and Y0 <= y <= Y1)


def boundary_side(x, y, th):
    """回傳 None (沒看到邊界) 或 'LEFT'/'RIGHT'/'CENTER'。"""
    c, s = math.cos(th), math.sin(th)
    left = right = 0
    for d, l in ROI_PTS:
        px, py = x + d * c - l * s, y + d * s + l * c
        if outside(px, py):
            if l > 0:
                left += 1
            elif l < 0:
                right += 1
            else:
                left += 0.5
                right += 0.5
    if left + right == 0:
        return None
    if left > 1.5 * right:
        return 'LEFT'
    if right > 1.5 * left:
        return 'RIGHT'
    return 'CENTER'


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def simulate(balls, angle_deg, record=False):
    x, y, th = hc.START_POSE
    balls = list(balls)
    t = 0.0
    path = [(x, y)] if record else None
    picked_times = []
    bounces = 0
    while t < TIMEOUT and balls:
        # 1) 看得到的球 (最近的那顆)
        best = None
        for i, (bx, by) in enumerate(balls):
            dx, dy = bx - x, by - y
            dist = math.hypot(dx, dy)
            bearing = wrap(math.atan2(dy, dx) - th)
            if dist <= DETECT_R and abs(bearing) <= HALF_FOV and (best is None or dist < best[0]):
                best = (dist, bearing, i)
        if best is not None:
            dist, bearing, i = best
            t += abs(bearing) / TURN_W + 0.5          # 原地對準 (加 0.5 s 收斂)
            th = wrap(th + bearing)
            t += dist / APPROACH_V                     # 直走過去
            bx, by = balls.pop(i)
            x, y = bx + BLIND_EXTRA * math.cos(th), by + BLIND_EXTRA * math.sin(th)
            t += BLIND_EXTRA / APPROACH_V
            picked_times.append(t)
            if record:
                path.append((x, y))
            continue
        # 2) 邊界
        side = boundary_side(x, y, th)
        if side is not None:
            sign = -1.0 if side == 'LEFT' else 1.0
            th = wrap(th + sign * math.radians(angle_deg))
            t += math.radians(angle_deg) / TURN_W
            bounces += 1
            # 轉完如果還是看到邊界 (角落)，下一輪會再轉
            if bounces > 5000:
                break
            continue
        # 3) 直走
        x += CRUISE_V * DT * math.cos(th)
        y += CRUISE_V * DT * math.sin(th)
        t += DT
        if record and (len(path) == 0 or math.hypot(path[-1][0] - x, path[-1][1] - y) > 0.2):
            path.append((x, y))
    return {'done': not balls, 'time': t if not balls else None, 'picked': len(picked_times),
            'bounces': bounces, 'path': path}


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    angles = [float(a) for a in (sys.argv[2].split() if len(sys.argv) > 2 else ['90', '110', '135', '180'])]
    names = [f'ball_{i}' for i in range(1, 11)]
    layouts = [list(random_positions_half_court(random.Random(seed), names).values())
               for seed in range(1000, 1000 + n)]
    print(f'{n} 個隨機佈局 (10 顆球)，逾時 {TIMEOUT:.0f}s')
    summary = {}
    for a in angles:
        res = [simulate(b, a) for b in layouts]
        done = [r for r in res if r['done']]
        times = sorted(r['time'] for r in done)
        summary[a] = res
        med = times[len(times) // 2] if times else float('nan')
        mean = sum(times) / len(times) if times else float('nan')
        print(f'  轉 {a:5.0f}°：全部撿完 {len(done):3d}/{n} ({100 * len(done) / n:5.1f}%)，'
              f'撿完時間 中位數 {med:6.1f}s 平均 {mean:6.1f}s，平均撿到 {sum(r["picked"] for r in res) / n:4.1f} 顆，'
              f'平均轉向 {sum(r["bounces"] for r in res) / n:5.1f} 次')

    # 畫 Gazebo 用的三個佈局 (seed 501~503) 的軌跡
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.rcParams['font.sans-serif'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
        seeds = [501, 502, 503]
        fig, axes = plt.subplots(len(seeds), len(angles), figsize=(4 * len(angles), 4 * len(seeds)))
        for r_i, seed in enumerate(seeds):
            balls = list(random_positions_half_court(random.Random(seed), names).values())
            for c_i, a in enumerate(angles):
                ax = axes[r_i][c_i]
                res = simulate(balls, a, record=True)
                for color, d in hc.LINES.items():
                    (xa, ya), (xb, yb) = d['seg']
                    ax.plot([xa, xb], [ya, yb], color={'RED': 'red', 'BLUE': 'blue', 'MAGENTA': 'magenta',
                                                       'CYAN': 'darkturquoise'}[color], lw=2)
                px, py = zip(*res['path'])
                ax.plot(px, py, lw=0.6, color='tab:blue')
                ax.scatter([b[0] for b in balls], [b[1] for b in balls], s=25, c='yellow', edgecolors='k', zorder=3)
                title = f'{a:.0f}°  seed {seed}: ' + (f'{res["time"]:.0f}s' if res['done'] else f'{res["picked"]}/10 逾時')
                ax.set_title(title, fontsize=9)
                ax.set_aspect('equal')
                ax.set_xlim(-12.5, -0.5)
                ax.set_ylim(-6, 6)
                ax.tick_params(labelsize=6)
        plt.tight_layout()
        out = Path('/home/sean/ros2_ws/experiments/實驗數據/bounce_2d_sim_paths.png')
        plt.savefig(out, dpi=110)
        print(f'軌跡圖: {out}')
    except ImportError:
        pass


if __name__ == '__main__':
    main()
