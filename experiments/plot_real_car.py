"""實體車模式的模擬 (run_real_car.sh) 畫軌跡：黃毛線範圍、球、Gazebo 真實軌跡、EDGE / 轉向 / 撿到球的位置。

用法: python3 experiments/plot_real_car.py <runN> [layout.json]   輸出 runN/real_car_traj.png
"""
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src' / 'tennis_bot' / 'tennis_bot'))
import half_court as H  # noqa: E402

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def read(p):
    return list(csv.DictReader(open(p))) if Path(p).exists() else []


def main():
    run = ROOT / 'experiments' / '實驗數據' / sys.argv[1]
    layout = json.load(open(sys.argv[2])) if len(sys.argv) > 2 else {}
    layout = layout.get('positions', layout)
    tr = read(run / 'real_traj.csv')
    ev = read(run / 'stm32_events.csv')
    touches = read(run / 'ball_touches.csv')
    t = np.array([float(r['sim_t']) for r in tr])
    x = np.array([float(r['real_x']) for r in tr])
    y = np.array([float(r['real_y']) for r in tr])
    fig, ax = plt.subplots(figsize=(9, 7.5))
    yx = [H.YARN_X0, H.YARN_X1, H.YARN_X1, H.YARN_X0, H.YARN_X0]
    yy = [-H.YARN_Y, -H.YARN_Y, H.YARN_Y, H.YARN_Y, -H.YARN_Y]
    ax.plot(yx, yy, color='#d4b000', lw=3, label='黃毛線')
    ax.plot([H.BASELINE_X, 0, 0, H.BASELINE_X, H.BASELINE_X],
            [-H.SIDELINE_Y, -H.SIDELINE_Y, H.SIDELINE_Y, H.SIDELINE_Y, -H.SIDELINE_Y], color='#999', lw=1, ls='--',
            label='場地白線 / 網子')
    picked = {r['ball_name'] for r in touches}
    for name, (bx, by) in layout.items():
        ax.plot(bx, by, 'o', ms=9, color='#7cb342' if name in picked else '#c62828')
        ax.text(bx + 0.12, by + 0.12, name.replace('ball_', ''), fontsize=8)
    ax.plot(x, y, color='#1565c0', lw=1.2, label='車子 (Gazebo 真實位置)')
    if len(x):
        ax.plot(x[0], y[0], 's', color='k', ms=8, label='起點')
    for e in ev:
        if e['event'] != 'EDGE' or not len(t):
            continue
        te = float(e['sim_t'])
        ex, ey = np.interp(te, t, x), np.interp(te, t, y)
        ax.plot(ex, ey, 'x', color='#e65100', ms=9, mew=2)
    ax.plot([], [], 'x', color='#e65100', ms=9, mew=2, label='碰到黃線 (EDGE)')
    ax.plot([], [], 'o', color='#7cb342', label='撿到的球')
    ax.plot([], [], 'o', color='#c62828', label='沒撿到的球')
    res = (run / 'result.txt').read_text().replace('\n', '  ') if (run / 'result.txt').exists() else ''
    meta = (run / 'meta.txt').read_text() if (run / 'meta.txt').exists() else ''
    world = next((l.split('=', 1)[1] for l in meta.splitlines() if l.startswith('world=')), '?')
    ax.set_title(f'{sys.argv[1]}  {world}\n{res}', fontsize=10)
    ax.set_aspect('equal')
    ax.set_xlim(H.BASELINE_X - 1.5, 1.0)
    ax.set_ylim(-H.SIDELINE_Y - 1.5, H.SIDELINE_Y + 1.5)
    ax.set_xlabel('x (m)，網子在 x=0')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(run / 'real_car_traj.png', dpi=110)
    print(run / 'real_car_traj.png')


if __name__ == '__main__':
    main()
