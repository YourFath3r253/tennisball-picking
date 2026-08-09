"""固定的隨機球場佈局 - 3 組，每組 10 顆球的 (x, y) 位置。

用固定亂數種子產生一次就存起來，讓所有 (roller_omega, blind_speed) 參數組合
都在同樣 3 組佈局上測試，這樣比較才公平（不會有的組合剛好抽到比較好撿的佈局）。

放置半徑：實測校正過，相機沒有俯角，只有距離機器人約 0.5~1.2m 的球才會被
vision_node 的顏色偵測看到（太近會被鏡頭「看過頭」看不到、太遠球太小偵測不穩），
所以球放在以機器人起始點為圓心、半徑 0.5~1.2m 的環形區域內，360 度隨機角度
（車體搜尋時會原地自轉，各方向都掃得到）。球心之間至少間隔 0.4m。
"""
import math
import random

NUM_BALLS = 10
RADIUS_RANGE = (0.5, 1.2)
MIN_BALL_SEPARATION = 0.4

SEEDS = [1001, 1002, 1003]


def generate_layout(seed):
    rng = random.Random(seed)
    positions = []
    attempts = 0
    while len(positions) < NUM_BALLS and attempts < 10000:
        attempts += 1
        r = rng.uniform(*RADIUS_RANGE)
        theta = rng.uniform(0, 2 * math.pi)
        x, y = r * math.cos(theta), r * math.sin(theta)
        if any(((x - px) ** 2 + (y - py) ** 2) ** 0.5 < MIN_BALL_SEPARATION for px, py in positions):
            continue
        positions.append((x, y))
    if len(positions) < NUM_BALLS:
        raise RuntimeError(f'only placed {len(positions)}/{NUM_BALLS} balls for seed {seed}, loosen constraints')
    return positions


LAYOUTS = [generate_layout(seed) for seed in SEEDS]


if __name__ == '__main__':
    for i, layout in enumerate(LAYOUTS):
        print(f'layout {i} (seed={SEEDS[i]}):')
        for x, y in layout:
            print(f'  ({x:.2f}, {y:.2f})')
