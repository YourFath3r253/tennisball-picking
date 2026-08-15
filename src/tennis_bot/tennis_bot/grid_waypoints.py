"""8 欄 x 4 排 = 32 格弓字型網格。

編號規則：每排都是固定「由左到右」編號 (row0: 1-8, row1: 9-16, row2: 17-24,
row3: 25-32)，跟走的方向無關。但車體實際走訪順序是蛇形：
row0 由左到右走 (1->8)，row1 由右到左走 (16->9)，
row2 由左到右走 (17->24)，row3 由右到左走 (32->25)。
"""
import math


def generate_grid_waypoints(x_range, y_range, cols, rows):
    """回傳 (waypoints, cell_numbers, cell_w, cell_h)。
    waypoints[i] 跟 cell_numbers[i] 都是依「走訪順序」排列，
    cell_numbers 是對應的格子編號 (照上面固定的左到右編號規則)。
    """
    width = x_range[1] - x_range[0]
    height = y_range[1] - y_range[0]
    cell_w = width / cols
    cell_h = height / rows

    waypoints = []
    cell_numbers = []
    for r in range(rows):
        y = y_range[0] + (r + 0.5) * cell_h
        col_order = range(cols) if r % 2 == 0 else reversed(range(cols))
        for c in col_order:
            x = x_range[0] + (c + 0.5) * cell_w
            waypoints.append((x, y))
            cell_numbers.append(r * cols + c + 1)

    return waypoints, cell_numbers, cell_w, cell_h


if __name__ == '__main__':
    wps, nums, cw, ch = generate_grid_waypoints((-11.0, 11.0), (-4.5, 4.5), 8, 4)
    print(f'{len(wps)} 格，每格 {cw:.3f} x {ch:.3f} m')
    print('走訪順序 (格號):', nums)
    for n, (x, y) in zip(nums, wps):
        print(f'  格 {n}: ({x:.3f}, {y:.3f})')
