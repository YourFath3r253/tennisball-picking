"""驗證四色邊界偵測的觸發距離：車子面向某條色帶，放在不同距離，看 /court_boundary 什麼時候變 EDGE。

用法 (Gazebo 開 half_court_lines.world + vision_node)：
  python3 experiments/boundary_detect_test.py
對每條色帶 (正對) + 一個斜 45° 的情況，距離從 1.0 m 掃到 0.3 m (相機到色帶中心線)，印出偵測結果。
"""
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import SetEntityState

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src' / 'tennis_bot' / 'tennis_bot'))
import half_court as hc  # noqa: E402

CAM_IN_BASE = (0.10, -0.135)
DISTS = [1.0, 0.8, 0.7, 0.65, 0.6, 0.55, 0.5, 0.4, 0.3]


class T(Node):
    def __init__(self):
        super().__init__('boundary_detect_test')
        self.sc = self.create_client(SetEntityState, '/set_entity_state')
        self.sc.wait_for_service()
        self.msg = None
        self.create_subscription(String, '/court_boundary', lambda m: setattr(self, 'msg', m.data), 10)

    def place(self, cam_x, cam_y, yaw):
        # 給相機位置，反推 base_link 位置
        bx = cam_x - (CAM_IN_BASE[0] * math.cos(yaw) - CAM_IN_BASE[1] * math.sin(yaw))
        by = cam_y - (CAM_IN_BASE[0] * math.sin(yaw) + CAM_IN_BASE[1] * math.cos(yaw))
        r = SetEntityState.Request()
        r.state.name = 'tennis_bot'
        r.state.pose.position.x, r.state.pose.position.y, r.state.pose.position.z = bx, by, 0.02
        r.state.pose.orientation.z, r.state.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        r.state.reference_frame = 'world'
        f = self.sc.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)

    def read(self, settle=1.5):
        t0 = time.time()
        while time.time() - t0 < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.msg

    def run(self):
        # (名稱, 色帶位置, 車子朝向, 怎麼由距離算相機位置)
        cases = [
            ('RED 底線 正對', lambda d: (hc.BASELINE_X + d, 0.0, math.pi)),
            ('CYAN 網前線 正對', lambda d: (hc.NET_LINE_X - d, 0.0, 0.0)),
            ('BLUE 左邊線 正對', lambda d: (-6.0, hc.SIDELINE_Y - d, math.pi / 2)),
            ('MAGENTA 右邊線 正對', lambda d: (-6.0, -hc.SIDELINE_Y + d, -math.pi / 2)),
            ('BLUE 左邊線 斜45°(線在左前)', lambda d: (-6.0, hc.SIDELINE_Y - d, math.pi / 4)),
        ]
        for name, pose_of in cases:
            print(f'== {name}', flush=True)
            for d in DISTS:
                x, y, yaw = pose_of(d)
                self.place(x, y, yaw)
                print(f'  相機到色帶中心 {d:.2f} m -> {self.read()}', flush=True)


rclpy.init()
T().run()
rclpy.shutdown()
