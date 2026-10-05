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
