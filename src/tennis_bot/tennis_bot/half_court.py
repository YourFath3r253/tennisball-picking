"""D 同學的路徑規劃提案：半場、四條邊界各貼一種顏色，相機看到邊界就轉向。

這個檔案集中放半場幾何 + 四色邊界的定義，world 產生器、vision_node、boundary_bounce_node、
畫圖程式都從這裡讀，確保四個地方用的是同一組數字。

座標跟全場模擬一樣：網子在 x=0，我方半場是 x<0，面向網子時左手邊是 +y。
"""

# ITF 雙打場地：底線到網子 11.885 m，雙打寬 10.97 m (邊線在 y=±5.485)
BASELINE_X = -11.885
NET_X = 0.0
SIDELINE_Y = 5.485
# 網子不是畫在地上的線，在網子前 1 m 貼第四條色帶當「網子邊界」(看到就轉向，車身不會靠近網子)
NET_LINE_X = -1.0
LINE_WIDTH = 0.05  # 色帶寬 5 cm (ITF 邊線寬 5 cm)

# 四條邊界、四種顏色。HSV 是 OpenCV 範圍 (H 0~180)，色相要避開網球的黃綠色 (H 20~40)。
# x0,x1,y0,y1 是色帶中心線的兩端。
LINES = {
    'RED': {'label': '底線', 'rgb': (1.0, 0.0, 0.0),
            'hsv': [((0, 120, 60), (8, 255, 255)), ((172, 120, 60), (180, 255, 255))],
            'seg': ((BASELINE_X, -SIDELINE_Y), (BASELINE_X, SIDELINE_Y))},
    'BLUE': {'label': '左邊線', 'rgb': (0.0, 0.0, 1.0),
             'hsv': [((110, 120, 60), (130, 255, 255))],
             'seg': ((BASELINE_X, SIDELINE_Y), (NET_LINE_X, SIDELINE_Y))},
    'MAGENTA': {'label': '右邊線', 'rgb': (1.0, 0.0, 1.0),
                'hsv': [((140, 120, 60), (160, 255, 255))],
                'seg': ((BASELINE_X, -SIDELINE_Y), (NET_LINE_X, -SIDELINE_Y))},
    'CYAN': {'label': '網前線', 'rgb': (0.0, 1.0, 1.0),
             'hsv': [((80, 120, 60), (100, 255, 255))],
             'seg': ((NET_LINE_X, -SIDELINE_Y), (NET_LINE_X, SIDELINE_Y))},
}

# 起點：右後角 (底線+右邊線的角落往內縮)，朝向網子 (yaw=0)。是 base_link 的位置。
START_POSE = (-10.9, -4.6, 0.0)

# 球的擺放範圍：四條線往內縮 0.8 m，起點附近 1.5 m 內不放 (一開場就撿到測不到東西)
BALL_MARGIN_M = 0.8
BALL_START_EXCLUSION_M = 1.5


def ball_region():
    return ((BASELINE_X + BALL_MARGIN_M, NET_LINE_X - BALL_MARGIN_M),
            (-SIDELINE_Y + BALL_MARGIN_M, SIDELINE_Y - BALL_MARGIN_M))


def inside_lines(x, y, margin=0.0):
    """(x, y) 是否在四條線圍起來的範圍內 (往外放寬 margin 公尺)。"""
    return (BASELINE_X - margin <= x <= NET_LINE_X + margin
            and -SIDELINE_Y - margin <= y <= SIDELINE_Y + margin)

# ---- 2026-10-12 實體車現況：邊界改成「只有黃色毛線」(D 同學 yellow_runtime，沒有四色) ----
# 室內網球場 (indoor_court.world，experiments/gen_indoor_court_world.py 產生)：沿半場四邊拉一條黃毛線。
# 實際怎麼拉還沒確認，先假設在場地線外緣往內 YARN_INSET_M (底線寬 10 cm，離白線內緣還有 10 cm，不壓在白線上，D 的模型是看「線 vs 旁邊地面」的顏色差)。
# 毛線寬度照實拍 (yellow_sample.jpg 原始畫面)：0.33 m 處 4~14 px 寬，換算地面 6~20 mm (毛線蓬鬆 + 鏡頭模糊)，取中位數 14 mm。
YARN_INSET_M = 0.20
YARN_WIDTH_M = 0.014
YARN_HEIGHT_M = 0.001   # 做成扁條：4 mm 高的方條在 20 cm 高的鏡頭看來大多是側面，側面沒有頂燈照到會變暗 (實拍毛線比地面亮)
YARN_X0 = BASELINE_X + YARN_INSET_M      # 底線那條
YARN_X1 = NET_LINE_X                      # 網前那條 (網前 1 m)
YARN_Y = SIDELINE_Y - YARN_INSET_M        # 兩條邊線 y=±YARN_Y
YARN_SEGMENTS = {
    'baseline': ((YARN_X0, -YARN_Y), (YARN_X0, YARN_Y)),
    'left': ((YARN_X0, YARN_Y), (YARN_X1, YARN_Y)),
    'right': ((YARN_X0, -YARN_Y), (YARN_X1, -YARN_Y)),
    'net': ((YARN_X1, -YARN_Y), (YARN_X1, YARN_Y)),
}


def inside_yarn(x, y, margin=0.0):
    """(x, y) 是否在黃毛線圍起來的範圍內 (往外放寬 margin 公尺)。"""
    return (YARN_X0 - margin <= x <= YARN_X1 + margin and -YARN_Y - margin <= y <= YARN_Y + margin)
