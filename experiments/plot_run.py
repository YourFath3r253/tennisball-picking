"""畫某一次 run 的軌跡圖：車體路徑 (依狀態上色) + 全部 10 顆球位置 (綠色=撿到，
黃色紅框=沒撿到)。用法： python3 plot_run.py <run 資料夾路徑>
球的位置寫死在這裡，跟目前 tennis_court.world 用的佈局一致；如果之後
world 檔案換了新的隨機佈局，要記得更新這裡的 ALL_BALLS。
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

ALL_BALLS = {
    'ball_1': (-3.88, -3.14), 'ball_2': (3.32, -3.85), 'ball_3': (0.79, -1.21),
    'ball_4': (-9.72, 0.07), 'ball_5': (-10.18, -0.60), 'ball_6': (-9.46, -3.68),
    'ball_7': (-1.66, 2.94), 'ball_8': (-8.28, -2.49), 'ball_9': (2.80, 4.03),
    'ball_10': (1.70, -0.93),
}

STATE_COLORS = {'PATROL': 'tab:blue', 'ALIGN': 'orange', 'APPROACH': 'red',
                 'BLIND_DASH': 'purple', 'RECOVER': 'brown'}


def plot_run(run_dir):
    run_dir = Path(run_dir)
    traj_path = run_dir / 'trajectory.csv'
    touch_path = run_dir / 'ball_touches.csv'
    if not traj_path.exists():
        print(f'找不到 {traj_path}')
        return

    rows = list(csv.DictReader(open(traj_path)))
    if not rows:
        print('trajectory.csv 是空的，還沒有資料可以畫')
        return
    xs = [float(r['x']) for r in rows]
    ys = [float(r['y']) for r in rows]
    states = [r['state'] for r in rows]

    touches = list(csv.DictReader(open(touch_path))) if touch_path.exists() else []
    touched_names = {t['ball_name'] for t in touches}
    touch_time = {t['ball_name']: float(t['elapsed_sec']) for t in touches}

    fig, ax = plt.subplots(figsize=(14, 7))

    ax.add_patch(patches.Rectangle((-12, -5.5), 24, 11, linewidth=2, edgecolor='green',
                                    facecolor='none', label='Court wall'))
    ax.add_patch(patches.Rectangle((-11, -4.5), 22, 9, linewidth=1, edgecolor='gray',
                                    facecolor='none', linestyle='--', label='1m safety margin'))

    for state, color in STATE_COLORS.items():
        sx = [x for x, s in zip(xs, states) if s == state]
        sy = [y for y, s in zip(ys, states) if s == state]
        if sx:
            ax.scatter(sx, sy, s=6, c=color, label=state, zorder=3)

    ax.plot(xs, ys, '-', color='lightgray', linewidth=0.5, zorder=1)
    ax.scatter([xs[0]], [ys[0]], marker='*', s=300, c='black', label='Start', zorder=5)
    ax.scatter([xs[-1]], [ys[-1]], marker='X', s=200, c='black', label='End', zorder=5)

    first_t, first_m = True, True
    for name, (bx, by) in ALL_BALLS.items():
        if name in touched_names:
            ax.scatter([bx], [by], marker='o', s=120, facecolors='lime', edgecolors='black',
                       linewidths=1.5, zorder=6, label='Ball - touched' if first_t else None)
            ax.annotate(f'{name}\n(t={touch_time[name]:.1f}s)', (bx, by),
                        textcoords="offset points", xytext=(8, 8), fontsize=8)
            first_t = False
        else:
            ax.scatter([bx], [by], marker='o', s=120, facecolors='yellow', edgecolors='red',
                       linewidths=1.5, zorder=6, label='Ball - missed' if first_m else None)
            ax.annotate(name, (bx, by), textcoords="offset points", xytext=(8, 8), fontsize=8)
            first_m = False

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'{run_dir.name} - Trajectory + Balls ({len(touched_names)}/{len(ALL_BALLS)} touched)')
    ax.set_xlim(-13, 13)
    ax.set_ylim(-6.5, 6.5)
    ax.set_aspect('equal')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=4, fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = run_dir / 'trajectory_plot.png'
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'saved {out_path}')

    total_dist = sum(
        ((xs[i] - xs[i - 1]) ** 2 + (ys[i] - ys[i - 1]) ** 2) ** 0.5
        for i in range(1, len(xs))
    )
    print(f'touched: {len(touched_names)}/{len(ALL_BALLS)}')
    print(f'total logged distance: {total_dist:.1f} m')
    print(f'total time (last logged t): {rows[-1]["t"]} s')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('用法: python3 plot_run.py <run 資料夾路徑>')
        sys.exit(1)
    plot_run(sys.argv[1])
