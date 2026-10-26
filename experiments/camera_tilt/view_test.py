"""C1 + C3 (新車 v2 鏡頭仰角)：抓模擬畫面 + 球擺成網格跑 D 的 YOLO (同一個 ONNX、同一份前處理/距離估算)。

Gazebo 要先開著 (V2_CAM_PITCH_DEG=<仰角> WORLD=indoor_court.world bash experiments/v2_restart.sh)。
用法: python3 experiments/camera_tilt/view_test.py <out_dir> --tilt 21.3
輸出：
  view_open.png  車子在半場中間朝網子 (黃毛線在前方 ~7 m)
  view_line.png  車頭離網前黃毛線 ~0.6 m
  grid.csv       每個球位置一列：ground_d (球心到鏡頭正下方的水平距離 m)、bearing (度，正 = 右)、
                 detected、conf、D 估的 dist_cm / bearing_deg、真實斜距 slant_cm
"""
import argparse
import csv
import math
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

from tennis_bot.jetson_d_code import OnnxYolo, load_d_main

CAM_IN_BASE = (0.103, 0.0)
CAM_Z = 0.222
BALL_Z = 0.033
CAR = (-8.0, 0.0, 0.0)
DISTS = [0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.22, 0.25, 0.28, 0.32, 0.36, 0.40, 0.50, 0.60, 0.80, 1.0, 1.25,
         1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0]
BEARINGS = [-30, -20, -10, 0, 10, 20, 30]


class Grab(Node):
    def __init__(self):
        super().__init__('camera_tilt_view_test')
        self.img = None
        self.stamp = 0.0
        self.sim_t = 0.0
        self.br = CvBridge()
        self.create_subscription(Image, '/tennis_camera/image_raw', self._img, 1)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'sim_t', m.clock.sec + m.clock.nanosec * 1e-9),
                                 qos_profile_sensor_data)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.cli.wait_for_service()

    def _img(self, m):
        self.img = m
        self.stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def setp(self, name, x, y, z, yaw=0.0):
        r = SetEntityState.Request()
        r.state.name = name
        r.state.pose.position.x, r.state.pose.position.y, r.state.pose.position.z = float(x), float(y), float(z)
        r.state.pose.orientation.z, r.state.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        r.state.reference_frame = 'world'
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5)

    def fresh_frame(self, settle=0.15):
        """teleport 之後拿一張「拍攝時間晚於 teleport + settle」的畫面 (模擬時間)。"""
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.02)
        t_req = self.sim_t + settle
        while self.img is None or self.stamp < t_req:
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.br.imgmsg_to_cv2(self.img, 'bgr8')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--tilt', type=float, required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    d = load_d_main()
    yolo = OnnxYolo()
    rclpy.init()
    n = Grab()
    for i in range(2, 11):
        n.setp(f'ball_{i}', 30 + 0.3 * i, 30, BALL_Z)
    n.setp('ball_1', 30, 31, BALL_Z)
    x0, y0, yaw = CAR
    n.setp('tennis_bot', x0, y0, 0.01, yaw)
    cv2.imwrite(str(out / 'view_open.png'), n.fresh_frame(0.5))
    n.setp('tennis_bot', -1.0 - 0.007 - 0.248 - 0.6, 0.0, 0.01, 0.0)
    cv2.imwrite(str(out / 'view_line.png'), n.fresh_frame(0.5))
    n.setp('tennis_bot', x0, y0, 0.01, yaw)
    n.fresh_frame(0.3)
    cx = x0 + CAM_IN_BASE[0] * math.cos(yaw)
    cy = y0 + CAM_IN_BASE[0] * math.sin(yaw)
    with open(out / 'grid.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['tilt', 'ground_d', 'bearing', 'slant_cm', 'detected', 'conf', 'est_dist_cm', 'est_bearing_deg',
                    'box_cy_px', 'n_dets'])
        for bd in BEARINGS:
            for gd in DISTS:
                b = math.radians(bd)
                bx, by = cx + gd * math.cos(yaw - b), cy + gd * math.sin(yaw - b)
                n.setp('ball_1', bx, by, BALL_Z)
                frame = n.fresh_frame()
                inp, _ = d['preprocess'](frame)
                dets = d['parse_yolo_output'](yolo.infer(inp))
                slant = math.hypot(gd, CAM_Z - BALL_Z) * 100
                if dets:
                    cands = [(det, d['estimate_ball_position'](det, d['FOCAL_LENGTH_PX'])) for det in dets]
                    det, pos = min(cands, key=lambda c: abs(c[1]['dist_cm'] - slant))
                    w.writerow([a.tilt, gd, bd, f'{slant:.1f}', 1, f'{det["conf"]:.3f}', f'{pos["dist_cm"]:.1f}',
                                f'{pos["bearing_deg"]:.1f}', f'{pos["cy"]:.1f}', len(dets)])
                else:
                    w.writerow([a.tilt, gd, bd, f'{slant:.1f}', 0, '', '', '', '', 0])
                if bd == 0 and gd in (0.2, 0.3, 1.0, 3.0):
                    cv2.imwrite(str(out / f'ball_{gd:.1f}m.png'), frame)
    n.setp('ball_1', 30, 31, BALL_Z)
    print('done', out)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
