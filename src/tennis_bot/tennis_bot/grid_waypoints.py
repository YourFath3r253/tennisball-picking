"""8 欄 x 4 排 = 32 格弓字型網格。

編號規則：每排都是固定「由左到右」編號 (row0: 1-8, row1: 9-16, row2: 17-24,
row3: 25-32)，跟走的方向無關。但車體實際走訪順序是蛇形：
row0 由左到右走 (1->8)，row1 由右到左走 (16->9)，
row2 由左到右走 (17->24)，row3 由右到左走 (32->25)。
"""
import math

# ---- 開放式網球場 (沒有牆) 的巡邏範圍 = 標準雙打場地大小 24 x 11 m ----
COURT_X_RANGE = (-12.0, 12.0)
COURT_Y_RANGE = (-5.5, 5.5)

# ---- 網格大小由相機決定：格邊長 = 2 * 0.8 * R * sin(θ) ----
# R = 相機能穩定辨識球的距離 (vision_node 55px² 門檻 ≈ 3.0 m)，θ = 相機水平視角的一半
# (URDF horizontal_fov 1.396 rad = 80° -> θ = 40°)，0.8 是安全係數 (相鄰格子視野重疊，
# 不漏球)。車走在格子中心線上，左右各看得到 0.8*R*sinθ，剛好蓋滿整格。
CAMERA_RANGE_M = 3.0
# 2026-10-05 相機改成對應 D 同學實測：水平視角 66.6° -> θ = 33.3° (原本 80° -> 40°)，格邊長 3.09 -> 2.63 m
CAMERA_HALF_FOV_DEG = math.degrees(math.atan(256.0 / 390.0))
COVERAGE_SAFETY = 0.8
CELL_MAX_M = 2 * COVERAGE_SAFETY * CAMERA_RANGE_M * math.sin(math.radians(CAMERA_HALF_FOV_DEG))


# ---- 網子：在 x=0 橫跨球場，網柱在 y=±6.4 (雙打邊線外 0.914 m) ----
NET_X = 0.0
NET_POST_Y = 6.4
# 車體中心 (里程計點) 離網子至少這麼遠：規則是車身離網 0.5 m，車頭在中心前方 0.263 m，
# 再留一點餘裕 -> 0.9 m
NET_CLEARANCE_M = 0.9
# 繞網柱過去時走的 y (網柱外 1.1 m：里程計航向誤差幾度時位置會差到 1 m，留餘裕不撞網柱)
NET_BYPASS_Y = NET_POST_Y + 1.1


def court_path_with_net():
    """有網子的球場：Choset boustrophedon cellular decomposition 的最簡單特例——
    網子把場地切成兩個沒有障礙物的矩形 cell，各自走弓字，中間繞網柱外側過去。
    回傳 (waypoints, cell_numbers, cells)：
      waypoints/cell_numbers 依走訪順序 (過渡點的 cell_number = 0)，
      cells = [(x0, y0, w, h, number), ...] 給畫圖用。"""
    half1_x = (COURT_X_RANGE[0], NET_X - NET_CLEARANCE_M)
    half2_x = (NET_X + NET_CLEARANCE_M, COURT_X_RANGE[1])
    cols, rows = grid_dims(half1_x, COURT_Y_RANGE)

    wps1, nums1, cw, ch = generate_grid_waypoints(half1_x, COURT_Y_RANGE, cols, rows)
    wps2, nums2, _, _ = generate_grid_waypoints(half2_x, COURT_Y_RANGE, cols, rows)
    # 第二半場從「靠網子、跟第一半場結束那排同側」的角落開始：把走訪順序整個反過來
    # (原本從左下開始、右上結束 -> 變成右上開始)，這樣繞過網柱後就直接接上。
    wps2 = list(reversed(wps2))
    nums2 = [n + cols * rows for n in reversed(nums2)]

    end1 = wps1[-1]
    bypass_y = NET_BYPASS_Y if end1[1] > 0 else -NET_BYPASS_Y
    transit = [(NET_X - NET_CLEARANCE_M - 1.0, bypass_y), (NET_X + NET_CLEARANCE_M + 1.0, bypass_y)]

    waypoints = wps1 + transit + wps2
    cell_numbers = nums1 + [0, 0] + nums2

    cells = []
    for half_i, x_range in enumerate((half1_x, half2_x)):
        for r in range(rows):
            for c in range(cols):
                cells.append((x_range[0] + c * cw, COURT_Y_RANGE[0] + r * ch, cw, ch,
                              half_i * cols * rows + r * cols + c + 1))
    return waypoints, cell_numbers, cells


def grid_dims(x_range, y_range, cell_max=CELL_MAX_M):
    """依格子最大邊長算出需要的欄數/排數 (無條件進位，格子只會比上限小)。"""
    cols = math.ceil((x_range[1] - x_range[0]) / cell_max)
    rows = math.ceil((y_range[1] - y_range[0]) / cell_max)
    return cols, rows


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
    cols, rows = grid_dims(COURT_X_RANGE, COURT_Y_RANGE)
    wps, nums, cw, ch = generate_grid_waypoints(COURT_X_RANGE, COURT_Y_RANGE, cols, rows)
    print(f'格邊長上限 {CELL_MAX_M:.3f} m -> {cols}x{rows}={len(wps)} 格，每格 {cw:.3f} x {ch:.3f} m')
    print('走訪順序 (格號):', nums)
    for n, (x, y) in zip(nums, wps):
        print(f'  格 {n}: ({x:.3f}, {y:.3f})')
