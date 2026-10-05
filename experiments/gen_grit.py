"""在 world 檔案裡產生「輕微砂礫」：場地上隨機撒很多個凸出地面幾毫米的小顆粒 (模擬真實化 R3)。

每顆砂礫是一顆半埋在地面下的小球 (靜態、有碰撞)，只露出頂端 h 毫米，車輪/萬向輪/網球
壓過去會被輕微頂起、方向被擾動，用來模擬真實球場的砂粒、小石頭、落葉梗這類地面不平。
固定亂數種子，每次產生的位置都一樣 (實驗可重現)。

用法:
  python3 experiments/gen_grit.py --world src/tennis_bot/worlds/tennis_court.world
  python3 experiments/gen_grit.py --world ... --remove      # 拿掉砂礫
"""
import argparse
import random
import re
from pathlib import Path

DEFAULT_WORLD = Path(__file__).resolve().parent.parent / 'src' / 'tennis_bot' / 'worlds' / 'tennis_court.world'

# 撒砂礫的範圍：整個球場再往外 1 m (車子會開到線外一點點)
X_RANGE = (-13.0, 13.0)
Y_RANGE = (-6.5, 6.5)
DENSITY_PER_M2 = 1.0          # 每平方公尺 1 顆
PROTRUDE_MM = (1.0, 4.0)      # 露出地面高度 1~4 mm
RADIUS_MM = (4.0, 8.0)        # 顆粒半徑 4~8 mm (只露出頂端一小段，像一顆小凸起)
SEED = 2026

BLOCK_RE = re.compile(r'[ \t]*<!-- 輕微砂礫 .*?</model>\n', re.S)


def grit_block(seed, density, x_range, y_range):
    rng = random.Random(seed)
    area = (x_range[1] - x_range[0]) * (y_range[1] - y_range[0])
    n = int(round(area * density))
    lines = [
        f'    <!-- 輕微砂礫 (模擬真實化 R3，experiments/gen_grit.py 產生，seed={seed})：{n} 顆半埋小球，'
        f'露出 {PROTRUDE_MM[0]:.0f}~{PROTRUDE_MM[1]:.0f} mm -->',
        '    <model name="court_grit">',
        '      <static>true</static>',
        '      <link name="grit_link">',
    ]
    for i in range(n):
        x = rng.uniform(*x_range)
        y = rng.uniform(*y_range)
        r = rng.uniform(*RADIUS_MM) / 1000.0
        h = rng.uniform(*PROTRUDE_MM) / 1000.0
        z = h - r  # 球心在地面下，只露出頂端 h
        geom = f'<geometry><sphere><radius>{r:.4f}</radius></sphere></geometry>'
        lines.append(f'        <collision name="g{i}_c"><pose>{x:.3f} {y:.3f} {z:.4f} 0 0 0</pose>{geom}</collision>')
        lines.append(f'        <visual name="g{i}_v"><pose>{x:.3f} {y:.3f} {z:.4f} 0 0 0</pose>{geom}'
                     '<material><ambient>0.35 0.33 0.3 1</ambient><diffuse>0.35 0.33 0.3 1</diffuse></material></visual>')
    lines += ['      </link>', '    </model>']
    return '\n'.join(lines) + '\n', n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default=str(DEFAULT_WORLD))
    ap.add_argument('--seed', type=int, default=SEED)
    ap.add_argument('--density', type=float, default=DENSITY_PER_M2)
    ap.add_argument('--x-range', type=float, nargs=2, default=X_RANGE)
    ap.add_argument('--y-range', type=float, nargs=2, default=Y_RANGE)
    ap.add_argument('--remove', action='store_true')
    args = ap.parse_args()

    path = Path(args.world)
    content = BLOCK_RE.sub('', path.read_text())
    if not args.remove:
        block, n = grit_block(args.seed, args.density, tuple(args.x_range), tuple(args.y_range))
        # 插在第一顆球前面 (沒有球就插在 </world> 前面)
        m = re.search(r'[ \t]*<model name="ball_\d+">', content)
        pos = m.start() if m else content.rindex('  </world>')
        content = content[:pos] + block + content[pos:]
        print(f'{path.name}: 加入 {n} 顆砂礫')
    else:
        print(f'{path.name}: 已移除砂礫')
    path.write_text(content)


if __name__ == '__main__':
    main()
