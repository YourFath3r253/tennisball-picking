"""畫某一次 run 的軌跡圖：車體「真實」路徑 (Gazebo 實測 real_x/real_y，依狀態
上色) + 里程計 (陀螺儀+輪速) 以為的路徑 (黑色虛線，疊上去比對兩者差多少) +
全部 10 顆球位置 (綠色=撿到，黃色紅框=沒撿到)。
用法： python3 plot_run.py <run 資料夾路徑>
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
from matplotlib.collections import LineCollection

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src' / 'tennis_bot' / 'tennis_bot'))
from grid_waypoints import generate_grid_waypoints

X_RANGE = (-11.0, 11.0)
Y_RANGE = (-4.5, 4.5)
GRID_COLS = 8
GRID_ROWS = 4

ALL_BALLS = {
    'ball_1': (-3.88, -3.14), 'ball_2': (3.32, -3.85), 'ball_3': (0.79, -1.21),
    'ball_4': (-9.72, 0.07), 'ball_5': (-10.18, -0.60),
    'ball_7': (-1.66, 2.94), 'ball_8': (-8.28, -2.49), 'ball_9': (2.80, 4.03),
    'ball_10': (1.70, -0.93),
}

STATE_COLORS = {'PATROL': 'tab:blue', 'ALIGN': 'orange', 'APPROACH': 'red',
                 'BLIND_DASH': 'purple', 'RECOVER': 'brown'}


def plot_run(run_dir, extra_markers=None, balls=None):
    """extra_markers: 額外要標在圖上的點，例如死鎖保護觸發的位置。
    格式: [(x, y, label), ...]
    balls: 覆蓋預設的 ALL_BALLS，用在每個 run 球佈局不同的情況 (批次測試)。"""
    balls = balls if balls is not None else ALL_BALLS
    run_dir = Path(run_dir)
    traj_path = run_dir / 'trajectory.csv'
    touch_path = run_dir / 'ball_touches.csv'
    if not traj_path.exists():
        print(f'找不到 {traj_path}')
        return

    all_rows = list(csv.DictReader(open(traj_path)))
    if not all_rows:
        print('trajectory.csv 是空的，還沒有資料可以畫')
        return

    # 只畫 Gazebo 實測的真實位移，不畫里程計「以為的」路徑；沒有 real_x/real_y
    # 的舊 run 檔案就沒東西可畫。
    rows = [r for r in all_rows if r.get('real_x') and r.get('real_y')]
    if not rows:
        print('這個 run 沒有真實座標紀錄 (real_x/real_y)，沒東西可畫')
        return
    xs = [float(r['real_x']) for r in rows]
    ys = [float(r['real_y']) for r in rows]
    states = [r['state'] for r in rows]
    # 里程計(陀螺儀+輪速)以為的位置，疊上去比對跟真實路徑差多少
    odo_xs = [float(r['x']) for r in rows]
    odo_ys = [float(r['y']) for r in rows]

    touches = list(csv.DictReader(open(touch_path))) if touch_path.exists() else []
    touched_names = {t['ball_name'] for t in touches}
    touch_time = {t['ball_name']: float(t['elapsed_sec']) for t in touches}

    fig, ax = plt.subplots(figsize=(14, 7))

    ax.add_patch(patches.Rectangle((-12, -5.5), 24, 11, linewidth=2, edgecolor='green',
                                    facecolor='none', label='Court wall'))
    ax.add_patch(patches.Rectangle((-11, -4.5), 22, 9, linewidth=1, edgecolor='gray',
                                    facecolor='none', linestyle='--', label='1m safety margin'))

    _, _, cell_w, cell_h = generate_grid_waypoints(X_RANGE, Y_RANGE, GRID_COLS, GRID_ROWS)
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            cx0 = X_RANGE[0] + c * cell_w
            cy0 = Y_RANGE[0] + r * cell_h
            ax.add_patch(patches.Rectangle((cx0, cy0), cell_w, cell_h, linewidth=0.6,
                                            edgecolor='silver', facecolor='none', zorder=0))
            cell_num = r * GRID_COLS + c + 1
            ax.text(cx0 + cell_w / 2, cy0 + cell_h / 2, str(cell_num),
                    ha='center', va='center', fontsize=7, color='silver', zorder=0)

    # 用線段連起來 (不是散點)，每一段依當下 state 上色，這樣狀態切換的地方顏色會
    # 自然轉換，又不會像分開畫散點那樣把不同時段的同一個 state 錯誤地連在一起。
    if len(xs) >= 2:
        segments = [[(xs[i], ys[i]), (xs[i + 1], ys[i + 1])] for i in range(len(xs) - 1)]
        seg_colors = [STATE_COLORS.get(states[i], 'gray') for i in range(len(xs) - 1)]
        ax.add_collection(LineCollection(segments, colors=seg_colors, linewidths=1.8, zorder=3))
        used_states = [s for s in STATE_COLORS if s in states]
        for s in used_states:
            ax.plot([], [], color=STATE_COLORS[s], linewidth=1.8, label=s)

    ax.plot(odo_xs, odo_ys, '--', color='black', linewidth=1.0, zorder=2,
            label='Odometry (gyro+wheel, believed)')

    ax.scatter([xs[0]], [ys[0]], marker='*', s=300, c='black', label='Start', zorder=5)
    ax.scatter([xs[-1]], [ys[-1]], marker='X', s=200, c='black', label='End', zorder=5)

    # 沿路徑每隔固定距離畫一個方向箭頭，標示走的先後順序
    cum_dist = [0.0]
    for i in range(1, len(xs)):
        cum_dist.append(cum_dist[-1] + ((xs[i] - xs[i - 1]) ** 2 + (ys[i] - ys[i - 1]) ** 2) ** 0.5)
    total_dist_for_arrows = cum_dist[-1]
    if total_dist_for_arrows > 0:
        arrow_spacing_m = 2.0
        next_target = arrow_spacing_m
        i = 1
        while i < len(xs) and next_target < total_dist_for_arrows:
            while i < len(xs) - 1 and cum_dist[i] < next_target:
                i += 1
            dx, dy = xs[i] - xs[i - 1], ys[i] - ys[i - 1]
            if dx != 0 or dy != 0:
                ax.annotate('', xy=(xs[i], ys[i]), xytext=(xs[i - 1], ys[i - 1]),
                             arrowprops=dict(arrowstyle='-|>', color='black', lw=0.8,
                                              mutation_scale=12, shrinkA=0, shrinkB=0),
                             zorder=4)
            next_target += arrow_spacing_m

    first_t, first_m = True, True
    for name, (bx, by) in balls.items():
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

    if extra_markers:
        for i, (mx, my, mlabel) in enumerate(extra_markers):
            ax.scatter([mx], [my], marker='X', s=220, facecolors='cyan', edgecolors='black',
                       linewidths=1.5, zorder=7, label='Livelock trigger' if i == 0 else None)
            ax.annotate(mlabel, (mx, my), textcoords="offset points", xytext=(8, -14),
                        fontsize=8, color='darkslategray', fontweight='bold')

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'{run_dir.name} - Real Trajectory + Balls ({len(touched_names)}/{len(balls)} touched)')
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
    print(f'touched: {len(touched_names)}/{len(balls)}')
    print(f'total real distance: {total_dist:.1f} m')
    print(f'total time (last logged t): {rows[-1]["t"]} s')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('用法: python3 plot_run.py <run 資料夾路徑>')
        sys.exit(1)
    plot_run(sys.argv[1])
