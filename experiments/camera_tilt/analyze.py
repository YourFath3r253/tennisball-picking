"""彙整鏡頭仰角掃描 (C2 邊界停車 + C3 看球網格)，印表格 + 畫圖。

用法: python3 experiments/camera_tilt/analyze.py <sweep_dir>
"""
import csv
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def edge_table(d):
    rows = [r for r in csv.DictReader(open(d / 'edge' / 'summary.csv')) if 'INVALID' not in r['result']]
    by = defaultdict(list)
    for r in rows:
        by[float(r['tilt'])].append(r)
    print('C2 邊界停車 (巡邏 120 rpm)：車頭離線距離 m (負 = 壓線/越線)')
    print('仰角  趟數 越線 沒EDGE  車頭最小:中位/最差   轉向時車體最小:最差')
    out = {}
    for t in sorted(by):
        rs = by[t]
        fm = [float(r['front_min_margin']) for r in rs if r['front_min_margin']]
        tm = [float(r['turn_min_margin']) for r in rs if r['turn_min_margin']]
        over = sum(1 for r in rs if 'NOSE_OVER' in r['result'])
        noedge = sum(1 for r in rs if r['result'].startswith('NO_EDGE'))
        out[t] = (fm, tm, over, len(rs))
        print(f'{t:5.1f}  {len(rs):3d}  {over:3d}  {noedge:4d}    {st.median(fm):+.2f} / {min(fm):+.2f}'
              f'           {min(tm):+.2f}')
    return rows, out


def grid_table(d):
    print('\nC3 看球 (D 的 YOLO)：正前方 (bearing 0) 偵測得到的地面距離範圍；括號 = D 估的距離 cm')
    res = {}
    for g in sorted(d.glob('view_tilt*/grid.csv'), key=lambda p: float(p.parent.name[9:])):
        rows = list(csv.DictReader(open(g)))
        t = float(rows[0]['tilt'])
        per_b = defaultdict(list)
        for r in rows:
            per_b[int(r['bearing'])].append(r)
        line = []
        lim = {}
        for b in sorted(per_b):
            rs = sorted(per_b[b], key=lambda r: float(r['ground_d']))
            det = [r for r in rs if r['detected'] == '1' and r['est_dist_cm'] and float(r['est_dist_cm']) < 3 * float(r['slant_cm'])]
            if not det:
                lim[b] = None
                continue
            near, far = det[0], det[-1]
            lim[b] = (float(near['ground_d']), float(far['ground_d']), float(near['est_dist_cm']))
        res[t] = lim
        n0 = lim.get(0)
        txt = '--' if not n0 else f'{n0[0]:.2f}~{n0[1]:.1f} m (最近時 D 估 {n0[2]:.0f} cm)'
        side = ' '.join(f'{b:+d}°:' + ('--' if not lim[b] else f'{lim[b][0]:.2f}~{lim[b][1]:.1f}')
                        for b in sorted(lim) if b != 0)
        print(f'{t:5.1f}°  正前 {txt}   {side}')
    return res


def main():
    d = Path(sys.argv[1])
    rows, out = edge_table(d)
    grid = grid_table(d)
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    angs = sorted({int(r['angle']) for r in rows})
    for a in angs:
        xs = [float(r['tilt']) for r in rows if int(r['angle']) == a]
        ys = [float(r['front_min_margin']) for r in rows if int(r['angle']) == a]
        ax[0].plot(xs, ys, 'o', alpha=0.7, label=f'撞線角度 {a:+d}°')
    ax[0].axhline(0, color='r', lw=1)
    ax[0].set_xlabel('鏡頭俯角 (°)')
    ax[0].set_ylabel('車頭停下時離黃線 (m)，負 = 越線')
    ax[0].set_title('C2 邊界停車 (新車 v2、巡邏 120 rpm、D 程式 SIL)')
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ts = sorted(grid)
    near = [grid[t][0][0] if grid[t].get(0) else None for t in ts]
    far = [grid[t][0][1] if grid[t].get(0) else None for t in ts]
    ax[1].plot(ts, near, 'o-', label='最近看得到 (球心到鏡頭正下方)')
    ax[1].plot(ts, far, 's-', label='最遠看得到')
    ax[1].set_xlabel('鏡頭俯角 (°)')
    ax[1].set_ylabel('地面距離 (m)')
    ax[1].set_title('C3 正前方的球 D 的 YOLO 看得到的範圍')
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(d / 'camera_tilt_summary.png', dpi=100)
    print('\n圖:', d / 'camera_tilt_summary.png')


if __name__ == '__main__':
    main()
