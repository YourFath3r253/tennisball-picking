"""量化某一次 run 的里程計誤差：
  - 里程計 (x,y,yaw) vs Gazebo 真實位置 (real_x, real_y, real_yaw)
  - 如果有 x_wall/y_wall 欄位 (真實時鐘 dt 的影子里程計) 也一起比
  - dt_debug.csv 的 sum(wall_dt)/sum(sim_dt) = 這次 run 的平均 1/即時率
用法: python3 experiments/odom_error_report.py <run 資料夾> [<run 資料夾> ...]
"""
import csv
import math
import sys
from pathlib import Path


def path_len(xs, ys):
    return sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def report(run_dir):
    run_dir = Path(run_dir)
    rows = [r for r in csv.DictReader(open(run_dir / 'trajectory.csv')) if r.get('real_x')]
    if not rows:
        print(f'{run_dir.name}: 沒有真實座標')
        return
    rx = [float(r['real_x']) for r in rows]
    ry = [float(r['real_y']) for r in rows]
    ryaw = [float(r['real_yaw']) for r in rows]
    real_len = path_len(rx, ry)

    print(f'=== {run_dir.name}  ({len(rows)} 筆, 最後 t={rows[-1]["t"]}s, 真實路徑長 {real_len:.2f} m)')

    dt_path = run_dir / 'dt_debug.csv'
    if dt_path.exists():
        w = s = 0.0
        for d in csv.DictReader(open(dt_path)):
            if d['sim_dt'] == '':
                continue
            w += float(d['wall_dt'])
            s += float(d['sim_dt'])
        if s > 0:
            print(f'  時間: 真實 {w:.1f}s / 模擬 {s:.1f}s = {w / s:.4f}  (即時率 {s / w:.3f})')

    variants = [('里程計 (x,y)', 'x', 'y', 'yaw')]
    if rows[0].get('x_wall'):
        variants.append(('影子:真實時鐘dt', 'x_wall', 'y_wall', 'yaw_wall'))
    for label, kx, ky, kyaw in variants:
        ox = [float(r[kx]) for r in rows]
        oy = [float(r[ky]) for r in rows]
        oyaw = [float(r[kyaw]) for r in rows]
        errs = [math.hypot(ox[i] - rx[i], oy[i] - ry[i]) for i in range(len(rows))]
        yaw_errs = [abs(math.degrees(wrap(oyaw[i] - ryaw[i]))) for i in range(len(rows))]
        olen = path_len(ox, oy)
        print(f'  [{label}] 路徑長 {olen:.2f} m, 里程計/真實 = {olen / real_len:.4f}')
        print(f'      位置誤差: 平均 {sum(errs) / len(errs):.3f} m, 最大 {max(errs):.3f} m, 最後 {errs[-1]:.3f} m')
        print(f'      航向誤差: 平均 {sum(yaw_errs) / len(yaw_errs):.2f}°, 最大 {max(yaw_errs):.2f}°, 最後 {yaw_errs[-1]:.2f}°')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for d in sys.argv[1:]:
        report(d)
