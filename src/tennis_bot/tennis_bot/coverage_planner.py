import math


def compute_lane_spacing(camera_hfov_rad, detection_range_m, overlap_ratio=0.2):
    """相機在有效辨識距離處掃過的地面寬度，乘上 (1-重疊率) 當作巡邏車道間距。"""
    swath_width = 2.0 * detection_range_m * math.tan(camera_hfov_rad / 2.0)
    return swath_width * (1.0 - overlap_ratio)


def generate_boustrophedon_waypoints(
    court_length_m,
    court_width_m,
    margin_ratio=1.3,
    camera_hfov_rad=1.396,
    detection_range_m=2.0,
    overlap_ratio=0.2,
    center=(0.0, 0.0),
):
    """產生覆蓋球場的弓字型 (boustrophedon) 航點。

    沿長軸 (x) 來回掃，車道沿短軸 (y) 堆疊，車道間距依相機水平視角
    + 有效辨識距離 + 重疊率算出，確保相機視野在場地內不留死角。
    地圖邊界依 margin_ratio 放大，讓車能撿到場邊界外的球
    (對應 generate_map.py 產生地圖時的 1.3 倍留白邏輯)。

    回傳: (waypoints, lane_spacing)
      waypoints: [(x, y, yaw), ...] 依巡邏順序排列
      lane_spacing: 實際使用的車道間距 (m)
    """
    cx, cy = center
    half_len = (court_length_m * margin_ratio) / 2.0
    half_wid = (court_width_m * margin_ratio) / 2.0

    lane_spacing = compute_lane_spacing(camera_hfov_rad, detection_range_m, overlap_ratio)
    if lane_spacing <= 0:
        raise ValueError("lane_spacing 必須大於 0，檢查 FOV / detection_range 是否合理")

    num_lanes = max(1, math.ceil((2.0 * half_wid) / lane_spacing) + 1)
    actual_spacing = (2.0 * half_wid) / (num_lanes - 1) if num_lanes > 1 else 0.0

    waypoints = []
    going_positive_x = True
    for i in range(num_lanes):
        y = cy - half_wid + i * actual_spacing
        if going_positive_x:
            x_start, x_end, yaw = cx - half_len, cx + half_len, 0.0
        else:
            x_start, x_end, yaw = cx + half_len, cx - half_len, math.pi
        waypoints.append((x_start, y, yaw))
        waypoints.append((x_end, y, yaw))
        going_positive_x = not going_positive_x

    return waypoints, actual_spacing
