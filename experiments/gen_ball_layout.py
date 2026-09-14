"""產生隨機9顆球佈局，寫進 tennis_court.world (改 pose/mass/inertia)。

用法:
  python3 gen_ball_layout.py --mass 0.027 --seed 1 --save layouts/run1.json
      -> 隨機產生新的9個位置，存到 layouts/run1.json，同時寫進 world 檔案 (mass=0.027)
  python3 gen_ball_layout.py --mass 10.0 --load layouts/run1.json
      -> 讀 run1.json 的位置 (不重新隨機)，只是把 mass 換成 10.0，寫進 world 檔案

限制條件：離場地邊線至少1公尺、不能落在格1範圍內、球跟球至少間隔0.5公尺、
可選 --net-clearance 讓球離網子至少多遠。
"""
import argparse
import json
import random
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src' / 'tennis_bot' / 'tennis_bot'))
from grid_waypoints import court_path_with_net, COURT_X_RANGE, COURT_Y_RANGE, NET_X  # noqa: E402

# 巡邏網格跟 grid_patrol_node 同一套 (開放式場地 24x11 m，格數由相機視野公式算)。
# 球只放在場地內縮 1 公尺的範圍 (邊線附近的球現實上會滾出場外)。
X_RANGE = COURT_X_RANGE
Y_RANGE = COURT_Y_RANGE
WALL_MARGIN_M = 1.0
MIN_BALL_SPACING_M = 0.5

DEFAULT_BALL_NAMES = ['ball_1', 'ball_2', 'ball_3', 'ball_4', 'ball_5',
                      'ball_7', 'ball_8', 'ball_9', 'ball_10']

WORLD_PATH = Path(__file__).resolve().parent.parent / 'src' / 'tennis_bot' / 'worlds' / 'tennis_court.world'


def cell1_bounds():
    _, _, cells = court_path_with_net()
    x0, y0, cw, ch, _ = cells[0]  # 格 1 = 第一半場左下角，車的起點
    return (x0, x0 + cw), (y0, y0 + ch)


def in_cell1(x, y, c1x, c1y):
    return c1x[0] <= x <= c1x[1] and c1y[0] <= y <= c1y[1]


def random_positions(rng, net_clearance=0.0, names=DEFAULT_BALL_NAMES):
    c1x, c1y = cell1_bounds()
    lo_x, hi_x = X_RANGE[0] + WALL_MARGIN_M, X_RANGE[1] - WALL_MARGIN_M
    lo_y, hi_y = Y_RANGE[0] + WALL_MARGIN_M, Y_RANGE[1] - WALL_MARGIN_M
    positions = []
    attempts = 0
    while len(positions) < len(names):
        attempts += 1
        if attempts > 100000:
            raise RuntimeError('產生球位置太多次失敗，限制條件可能太嚴格')
        x = rng.uniform(lo_x, hi_x)
        y = rng.uniform(lo_y, hi_y)
        if in_cell1(x, y, c1x, c1y):
            continue
        if abs(x - NET_X) < net_clearance:  # 球離網子太近車撿不到 (車身不能靠近網子 0.5 m)
            continue
        if any((x - px) ** 2 + (y - py) ** 2 < MIN_BALL_SPACING_M ** 2 for px, py in positions):
            continue
        positions.append((round(x, 3), round(y, 3)))
    return dict(zip(names, positions))


BALL_TEMPLATE = """    <model name="{name}">
      <pose>{x} {y} 0.1 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>{mass}</mass>
          <inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>0.033</radius></sphere></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
            <bounce><restitution_coefficient>0.75</restitution_coefficient><threshold>0.01</threshold></bounce>
            <contact><ode><kp>{kp}</kp><kd>{kd}</kd><min_depth>0.001</min_depth></ode></contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>0.033</radius></sphere></geometry>
          <material><ambient>0.8 1.0 0.0 1</ambient><diffuse>0.8 1.0 0.0 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def write_world(positions, mass):
    """把 world 檔案裡所有 ball_* 模型整段換掉，改成 positions 裡的球 (球數任意)。"""
    inertia = round(0.4 * mass * 0.033 ** 2, 8)  # 實心球 I = 2/5 m r^2
    if mass <= 0.03:
        kp, kd = 100000.0, 1.0
    else:
        kp, kd = 100000000.0, 10.0

    content = WORLD_PATH.read_text()
    blocks = list(re.finditer(r'[ \t]*<model name="ball_\d+">.*?</model>\n', content, re.S))
    if not blocks:
        raise RuntimeError('world 檔案裡找不到 ball_* 模型，不知道要插在哪')
    new_blocks = ''.join(
        BALL_TEMPLATE.format(name=name, x=x, y=y, mass=mass, inertia=inertia, kp=kp, kd=kd)
        for name, (x, y) in positions.items()
    )
    content = content[:blocks[0].start()] + new_blocks + content[blocks[-1].end():]
    WORLD_PATH.write_text(content)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mass', type=float, required=True)
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--save', type=str, default=None, help='隨機產生新位置後存到這個json檔')
    ap.add_argument('--load', type=str, default=None, help='從這個json檔讀位置，不重新隨機')
    ap.add_argument('--net-clearance', type=float, default=0.0, help='球離網子 (x=0) 至少多遠，0=不限制')
    ap.add_argument('--count', type=int, default=None, help='球數 (ball_1..ball_N)；不給就用預設 9 顆的名字')
    args = ap.parse_args()

    if args.load:
        positions = json.loads(Path(args.load).read_text())
    else:
        rng = random.Random(args.seed)
        names = [f'ball_{i}' for i in range(1, args.count + 1)] if args.count else DEFAULT_BALL_NAMES
        positions = random_positions(rng, args.net_clearance, names)
        if args.save:
            Path(args.save).parent.mkdir(parents=True, exist_ok=True)
            Path(args.save).write_text(json.dumps(positions, indent=2, ensure_ascii=False))

    write_world(positions, args.mass)
    print(json.dumps({'positions': positions, 'mass': args.mass}, ensure_ascii=False))


if __name__ == '__main__':
    main()
