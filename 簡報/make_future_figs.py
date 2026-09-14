"""未來工作頁的兩張示意圖 (實體車轉向對準)：
(a) B 同學目前的方法：每次轉 5° 就停 1 秒再看相機，階梯式慢慢收斂
(b) 沒有等相機更新、增益太大時的發散/轉過頭
x 軸時間，y 軸球相對車體中心的夾角 (±40°)。純示意，不是量測數據。"""
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent / 'img'
START_DEG = 35.0
CAM_DELAY = 0.4  # 深度相機更新/辨識延遲 (s)，示意


def stairs_method(step_deg=5.0, pause=1.0, turn_time=0.25, t_end=12.0, dt=0.01):
    t, ang = 0.0, START_DEG
    ts, angs = [0.0], [ang]
    while t < t_end:
        if abs(ang) <= 2.0:  # 對準門檻
            t += dt; ts.append(t); angs.append(ang); continue
        d = -math.copysign(min(step_deg, abs(ang)), ang)
        n = int(turn_time / dt)
        for k in range(1, n + 1):
            t += dt; ts.append(t); angs.append(ang + d * k / n)
        ang += d
        n = int(pause / dt)
        for _ in range(n):
            t += dt; ts.append(t); angs.append(ang)
    return ts, angs


def divergent_p(kp=2.9, delay=CAM_DELAY, max_rate=80.0, t_end=12.0, dt=0.01):
    """P 控制，但相機回報的角度是 delay 秒前的：增益大時轉過頭、越擺越大"""
    ts, angs = [0.0], [START_DEG]
    ang = START_DEG
    hist = [(0.0, ang)]
    t = 0.0
    while t < t_end:
        seen = ang
        for ht, ha in reversed(hist):
            if ht <= t - delay:
                seen = ha; break
        rate = max(-max_rate, min(max_rate, -kp * seen * 2.0))
        ang += rate * dt
        ang = max(-40, min(40, ang))
        t += dt; ts.append(t); angs.append(ang); hist.append((t, ang))
    return ts, angs


def stable_pid(kp=0.9, kd=0.25, delay=CAM_DELAY, max_rate=60.0, t_end=12.0, dt=0.01):
    ts, angs = [0.0], [START_DEG]
    ang = START_DEG; hist = [(0.0, ang)]; t = 0.0; prev_seen = ang
    while t < t_end:
        seen = ang
        for ht, ha in reversed(hist):
            if ht <= t - delay:
                seen = ha; break
        d = (seen - prev_seen) / dt if t > 0 else 0.0
        prev_seen = seen
        rate = max(-max_rate, min(max_rate, -(kp * seen + kd * d)))
        ang += rate * dt
        t += dt; ts.append(t); angs.append(ang); hist.append((t, ang))
    return ts, angs


def plot(ts, angs, title, fname, extra=None):
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    ax.plot(ts, angs, color='black', linewidth=1.8, label=title)
    if extra:
        ax.plot(extra[0], extra[1], color='gray', linewidth=1.4, linestyle='--', label=extra[2])
    ax.axhline(0, color='gray', linewidth=0.8)
    ax.axhspan(-2, 2, color='lightgreen', alpha=0.35, label='對準門檻 ±2°')
    ax.set_ylim(-40, 40); ax.set_xlim(0, 12)
    ax.set_xlabel('time (s)'); ax.set_ylabel('ball angle from center (deg)')
    ax.set_title(title, fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc='upper right')
    fig.tight_layout()
    fig.savefig(OUT / fname, dpi=150)
    print('saved', OUT / fname)


if __name__ == '__main__':
    plt.rcParams['font.family'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
    ts, a = stairs_method()
    plot(ts, a, '(a) step 5° + pause 1 s (current real-robot method)', 'future_a_stairs.png')
    ts2, a2 = divergent_p()
    ts3, a3 = stable_pid()
    plot(ts2, a2, '(b) gain too high + camera delay: overshoot, diverges', 'future_b_divergent.png',
         extra=(ts3, a3, 'goal: tuned PID, stable step response'))
