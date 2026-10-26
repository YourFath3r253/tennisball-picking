"""把車擺到 (x, y, yaw)，球擺在相機前 ball_d 公尺 (可選)，抓一張 /tennis_camera/image_raw。
Gazebo 要先開著 (experiments/sim_up.sh)。相機在車上的位置依 TENNISBOT_ROBOT (v1 舊車 / v2 新車)。

用法: python3 experiments/grab_camera_frame.py out.png x y yaw_deg [ball_d_m] [ball_bearing_deg]
"""
import math
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from sensor_msgs.msg import Image

CAM_IN_BASE = {'v1': (0.20, -0.135), 'v2': (0.103, 0.0)}[os.environ.get('TENNISBOT_ROBOT', 'v1')]
OTHER_BALLS = ['ball_2', 'ball_3', 'ball_4', 'ball_5', 'ball_6', 'ball_7', 'ball_8', 'ball_9', 'ball_10']


class Grab(Node):
    def __init__(self):
        super().__init__('grab_camera_frame')
        self.img = None
        self.br = CvBridge()
        self.create_subscription(Image, '/tennis_camera/image_raw', lambda m: setattr(self, 'img', m), 1)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.cli.wait_for_service()

    def setp(self, name, x, y, z, yaw=0.0):
        r = SetEntityState.Request()
        r.state.name = name
        r.state.pose.position.x, r.state.pose.position.y, r.state.pose.position.z = float(x), float(y), float(z)
        r.state.pose.orientation.z, r.state.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        r.state.reference_frame = 'world'
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5)


def main():
    out, x, y, yaw = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), math.radians(float(sys.argv[4]))
    bd = float(sys.argv[5]) if len(sys.argv) > 5 else None
    bb = math.radians(float(sys.argv[6])) if len(sys.argv) > 6 else 0.0
    rclpy.init()
    n = Grab()
    for i, nm in enumerate(OTHER_BALLS):
        n.setp(nm, 8 + i * 0.3, 8, 0.033)
    n.setp('tennis_bot', x, y, 0.01, yaw)
    ox, oy = CAM_IN_BASE
    cx, cy = x + ox * math.cos(yaw) - oy * math.sin(yaw), y + ox * math.sin(yaw) + oy * math.cos(yaw)
    if bd:
        n.setp('ball_1', cx + bd * math.cos(yaw - bb), cy + bd * math.sin(yaw - bb), 0.033)
    else:
        n.setp('ball_1', 8, 9, 0.033)
    t = time.time()
    while time.time() - t < 2.5:
        rclpy.spin_once(n, timeout_sec=0.05)
    cv2.imwrite(out, n.br.imgmsg_to_cv2(n.img, 'bgr8'))
    print('saved', out)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
