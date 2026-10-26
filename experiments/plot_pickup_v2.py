"""把 pickup_test_v2.py 記錄的球心軌跡 (車體座標) 疊在新車體 v2 的側面剖面上。

用法: python3 experiments/plot_pickup_v2.py <pickup.csv> <out.png> [剖面 y=0.0]
剖面用 src/tennis_bot/urdf/meshes/v2/*.stl (公尺、base_link 座標) 跟平面 y=常數 的交線；
滾輪/驅動輪是 URDF 裡的圓柱，這裡畫成參考形狀。
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
MESH = ROOT / 'src' / 'tennis_bot' / 'urdf' / 'meshes' / 'v2'
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def read_stl(p):
    raw = p.read_bytes()
    n = int(np.frombuffer(raw[80:84], dtype=np.uint32)[0])
    rec = np.frombuffer(raw[84:84 + n * 50], dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return rec['v'].astype(float)


def section(tris, y0):
    segs = []
    s = tris[:, :, 1] - y0
    hit = (s.min(1) <= 0) & (s.max(1) >= 0)
    for tri, ss in zip(tris[hit], s[hit]):
        pts = []
        for i in range(3):
            a, b, sa, sb = tri[i], tri[(i + 1) % 3], ss[i], ss[(i + 1) % 3]
            if (sa <= 0 < sb) or (sb <= 0 < sa):
                pts.append(a + sa / (sa - sb) * (b - a))
        if len(pts) == 2:
            segs.append(pts)
    return segs


def main():
    csv_path, out = sys.argv[1], sys.argv[2]
    y0 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    fig, ax = plt.subplots(figsize=(13, 6))
    for p in sorted(MESH.glob('*.stl')):
        col = '#e65100' if 'part12' in p.name else '#607d8b' if 'sheet' in p.name else '#424242'
        for a, b in section(read_stl(p), y0):
            ax.plot([a[0], b[0]], [a[2], b[2]], color=col, lw=1)
    ax.add_patch(plt.Circle((0, 0.052), 0.052, fill=False, ls='--', color='k'))
    ax.axhline(0, color='brown', lw=1)
    traces = defaultdict(list)
    for r in csv.DictReader(open(csv_path)):
        traces[(r['trial'], r['ball'])].append((float(r['lx']), float(r['z']), float(r['t'])))
    cmap = plt.get_cmap('tab10')
    for k, ((trial, ball), tr) in enumerate(sorted(traces.items())):
        tr = [p for p in tr if -0.6 < p[0] < 0.7]
        if not tr:
            continue
        x, z, _ = zip(*tr)
        ax.plot(x, z, '-', color=cmap(k % 10), lw=1.5, label=f'{trial} {ball}')
        ax.plot(x[-1], z[-1], 'o', color=cmap(k % 10), ms=8)
        ax.add_patch(plt.Circle((x[-1], z[-1]), 0.0335, fill=False, color=cmap(k % 10)))
    ax.set_aspect('equal')
    ax.set_xlim(-0.6, 0.7)
    ax.set_ylim(-0.02, 0.32)
    ax.set_xlabel('車體座標 x (m，+ 車頭；0 = 驅動輪軸)')
    ax.set_ylabel('高度 z (m)')
    ax.set_title(f'新車體 v2 撿球道：球心軌跡 (中心剖面 y={y0:+.3f} m)；圓點/圓圈 = 最後位置')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    print(out)


if __name__ == '__main__':
    main()
