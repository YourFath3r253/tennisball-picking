"""C1 幾何部分：新車 v2 自己的車體 (CAD 網格 + 滾輪圓柱) 投影到鏡頭畫面，看每個仰角下車體擋住畫面多少。
不用 Gazebo；鏡頭內參照模擬 (640x480、水平 FOV 1.1626 rad)，位置照 URDF (0.103, 0, 0.2222)、左偏 3.96°。
不考慮前後遮擋 (只回答「畫面裡有沒有車體」)。

用法: python3 experiments/camera_tilt/self_view.py [仰角們...]   (預設 9 12 15 18 21.3 24 27 30)
"""
import math
import struct
import sys
from pathlib import Path

import numpy as np

MESH = Path(__file__).resolve().parents[2] / 'src' / 'tennis_bot' / 'urdf' / 'meshes' / 'v2'
CAM = np.array([0.103, 0.0, 0.2222])
YAW = 0.0691
W, H = 640, 480
FX = (W / 2) / math.tan(1.1626 / 2)
SKIP = ('camera', 'camerastand')
ROLLERS = [((0.1027, 0.0725, 0.0710), 0.7302), ((0.1027, -0.0726, 0.0710), 0.7302)]   # (xyz, pitch)，r 0.05 長 0.04


def stl_vertices(p):
    raw = p.read_bytes()
    n = struct.unpack('<I', raw[80:84])[0]
    rec = np.frombuffer(raw[84:84 + n * 50], dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return rec['v'].reshape(-1, 3).astype(float)


def roller_points():
    pts = []
    for (x, y, z), pitch in ROLLERS:
        ax = np.array([math.sin(pitch), 0.0, math.cos(pitch)])     # 繞 y 轉 pitch 後的圓柱軸 (原本 z)
        u = np.array([math.cos(pitch), 0.0, -math.sin(pitch)])
        v = np.array([0.0, 1.0, 0.0])
        for h in np.linspace(-0.02, 0.02, 5):
            for a in np.linspace(0, 2 * math.pi, 36, endpoint=False):
                pts.append(np.array([x, y, z]) + h * ax + 0.05 * (math.cos(a) * u + math.sin(a) * v))
    return np.array(pts)


def project(pts, tilt_deg):
    p = math.radians(tilt_deg)
    # 相機座標：x 前、y 左、z 上 (Gazebo 相機看 +x)；R = Rz(yaw) Ry(pitch)
    cz, sz, cp, sp = math.cos(YAW), math.sin(YAW), math.cos(p), math.sin(p)
    R = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    c = (pts - CAM) @ R          # 世界 -> 相機 (R 的轉置乘)
    front = c[:, 0] > 0.01
    c = c[front]
    u = W / 2 - FX * c[:, 1] / c[:, 0]
    v = H / 2 - FX * c[:, 2] / c[:, 0]
    inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    return u[inside], v[inside]


def main():
    tilts = [float(t) for t in sys.argv[1:]] or [9, 12, 15, 18, 21.3, 24, 27, 30]
    parts = {p.stem.replace('trunk_v2_', ''): stl_vertices(p) for p in sorted(MESH.glob('*.stl'))
             if not p.stem.endswith(SKIP)}
    parts['rollers'] = roller_points()
    for t in tilts:
        seen = []
        top_v = H
        for name, pts in parts.items():
            u, v = project(pts, t)
            if len(u):
                seen.append(f'{name}(v>={v.min():.0f}, u {u.min():.0f}~{u.max():.0f})')
                top_v = min(top_v, v.min())
        frac = (H - top_v) / H
        print(f'仰角 {t:5.1f}°：車體最高到畫面第 {top_v:.0f} 列 (底部 {100 * frac:.0f}% 高度內有車體) '
              + ('；'.join(seen) if seen else '沒有車體入鏡'))


if __name__ == '__main__':
    main()
