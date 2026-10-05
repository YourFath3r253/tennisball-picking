"""D 同學邊界提案的「轉向角度」細部研究 (2D 快速模擬，延伸 bounce_2d_sim.py)。

Sean 2026-10-05 的問題：文獻建議轉幾度？130/135/140 有差嗎？長邊/短邊用不同角度會不會比較好？
這裡比較三類規則 (400 個隨機佈局)：
  fixed     相對目前航向轉固定角度 (110~160°)
  percolor  依撞到哪條線 (顏色) 用不同角度：邊線 (BLUE/MAGENTA) vs 底線/網前線 (RED/CYAN)
  normal    用顏色知道撞到哪條線 + 陀螺儀絕對航向，轉到「朝場內的法線 ± phi」(固定或隨機)
並可加轉向誤差 TURN_NOISE_DEG (真實車轉向不會剛好準)。
用法: TURN_NOISE_DEG=3 python3 experiments/bounce_2d_angle_study.py [n_layouts] [球數]
結果見 experiments/實驗數據/bounce_2d_angle_study_results.md
"""

import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent  # ros2_ws
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


def turn_angle(angle, rng):
    """angle 是數字 = 固定角度；字串 'r110-160' = 每次在 110~160 度之間均勻隨機 (文獻上避免週期軌道的做法)。"""
    if isinstance(angle, str) and angle.startswith('r'):
        lo, hi = (float(v) for v in angle[1:].split('-'))
        return rng.uniform(lo, hi)
    return float(angle)


def simulate(balls, angle_deg, record=False, seed=0):
    rng = random.Random(seed)
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
            a = turn_angle(angle_deg, rng)
            th = wrap(th + sign * math.radians(a))
            t += math.radians(a) / TURN_W
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




# ===== Q3 追加：依顏色 (哪條線) 用不同轉角、以法線為基準的反彈 =====
import multiprocessing as mp

TURN_NOISE_DEG = float(__import__('os').environ.get('TURN_NOISE_DEG', '0'))
INWARD_NORMAL = {'RED': 0.0, 'CYAN': math.pi, 'BLUE': -math.pi / 2, 'MAGENTA': math.pi / 2}


def boundary_info(x, y, th):
    """回傳 (side, color)；color = ROI 裡超出最多的那條邊界。"""
    c, s = math.cos(th), math.sin(th)
    left = right = 0
    cnt = {'RED': 0, 'CYAN': 0, 'BLUE': 0, 'MAGENTA': 0}
    for d, l in ROI_PTS:
        px, py = x + d * c - l * s, y + d * s + l * c
        if outside(px, py):
            if px < X0: cnt['RED'] += 1
            if px > X1: cnt['CYAN'] += 1
            if py > Y1: cnt['BLUE'] += 1
            if py < Y0: cnt['MAGENTA'] += 1
            if l > 0: left += 1
            elif l < 0: right += 1
            else: left += 0.5; right += 0.5
    if left + right == 0:
        return None, None
    side = 'LEFT' if left > 1.5 * right else ('RIGHT' if right > 1.5 * left else 'CENTER')
    return side, max(cnt, key=cnt.get)


def new_heading(rule, th, side, color, rng):
    kind = rule[0]
    if kind == 'fixed':                      # 相對目前航向轉固定角度
        a = rule[1]
        return wrap(th + (-1.0 if side == 'LEFT' else 1.0) * math.radians(a)), a
    if kind == 'percolor':                   # 長邊/短邊不同角度：rule[1]=邊線(BLUE/MAGENTA)、rule[2]=底線/網前線
        a = rule[1] if color in ('BLUE', 'MAGENTA') else rule[2]
        return wrap(th + (-1.0 if side == 'LEFT' else 1.0) * math.radians(a)), a
    if kind == 'normal':                     # 用顏色知道撞到哪條線 + 陀螺儀絕對航向：轉到「內法線 ± phi」
        n = INWARD_NORMAL[color]
        phi = math.radians(rule[1]) if rule[1] is not None else math.radians(rng.uniform(-rule[2], rule[2]))
        if rule[1] is not None:
            # 往「遠離原本來的方向」那一側偏，避免原路回去
            phi = phi if wrap(th - (n + math.pi)) < 0 else -phi
        h = wrap(n + phi)
        return h, abs(math.degrees(wrap(h - th)))
    raise ValueError(rule)


def simulate_rule(balls, rule, seed=0):
    rng = random.Random(seed)
    x, y, th = hc.START_POSE
    balls = list(balls)
    t = 0.0
    bounces = 0
    while t < TIMEOUT and balls:
        best = None
        for i, (bx, by) in enumerate(balls):
            dx, dy = bx - x, by - y
            dist = math.hypot(dx, dy)
            bearing = wrap(math.atan2(dy, dx) - th)
            if dist <= DETECT_R and abs(bearing) <= HALF_FOV and (best is None or dist < best[0]):
                best = (dist, bearing, i)
        if best is not None:
            dist, bearing, i = best
            t += abs(bearing) / TURN_W + 0.5
            th = wrap(th + bearing)
            t += dist / APPROACH_V
            bx, by = balls.pop(i)
            x, y = bx + BLIND_EXTRA * math.cos(th), by + BLIND_EXTRA * math.sin(th)
            t += BLIND_EXTRA / APPROACH_V
            continue
        side, color = boundary_info(x, y, th)
        if side is not None:
            th, turned = new_heading(rule, th, side, color, rng)
            th = wrap(th + math.radians(rng.gauss(0.0, TURN_NOISE_DEG)))
            t += math.radians(turned) / TURN_W
            bounces += 1
            if bounces > 5000:
                break
            continue
        x += CRUISE_V * DT * math.cos(th)
        y += CRUISE_V * DT * math.sin(th)
        t += DT
    return (not balls), (t if not balls else None)


def _job(args):
    balls, rule, seed = args
    return simulate_rule(balls, rule, seed)


def q3(n_layouts, n_balls, rules):
    names = [f'ball_{i}' for i in range(1, n_balls + 1)]
    layouts = [list(random_positions_half_court(random.Random(s), names).values()) for s in range(1000, 1000 + n_layouts)]
    with mp.Pool(14) as pool:
        for label, rule in rules:
            res = pool.map(_job, [(b, rule, i) for i, b in enumerate(layouts)])
            done = sorted(r[1] for r in res if r[0])
            med = done[len(done) // 2] if done else float('nan')
            mean = sum(done) / len(done) if done else float('nan')
            p90 = done[int(0.9 * len(done))] if done else float('nan')
            print(f'  {label:28s} 撿完 {len(done):3d}/{n_layouts} ({100*len(done)/n_layouts:5.1f}%)  中位數 {med:6.1f}s  平均 {mean:6.1f}s  90百分位 {p90:6.1f}s', flush=True)


if __name__ == '__main__':
    n_layouts = int(sys.argv[1]) if len(sys.argv) > 1 else 400
    n_balls = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    fixed = [(f'固定 {a}°', ('fixed', a)) for a in (110, 120, 125, 130, 135, 140, 145, 150, 160)]
    percolor = [('邊線135/底線網前120', ('percolor', 135, 120)), ('邊線120/底線網前135', ('percolor', 120, 135)),
                ('邊線135/底線網前150', ('percolor', 135, 150)), ('邊線150/底線網前135', ('percolor', 150, 135))]
    normal = [('法線±30°', ('normal', 30, None)), ('法線±45°', ('normal', 45, None)), ('法線±60°', ('normal', 60, None)),
              ('法線+隨機±60°', ('normal', None, 60)), ('法線+隨機±80°', ('normal', None, 80))]
    print(f'== {n_balls} 顆球，{n_layouts} 個佈局，逾時 {TIMEOUT:.0f} s，轉向誤差 σ={TURN_NOISE_DEG}°')
    q3(n_layouts, n_balls, fixed + percolor + normal)
