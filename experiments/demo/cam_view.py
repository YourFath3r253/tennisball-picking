"""錄影用：車上鏡頭畫面 + D 程式送給 STM32 的最新 UART (BALL / COURT) + STM32 事件，疊字顯示在一個 OpenCV 視窗。
用法: python3 experiments/demo/cam_view.py [x y w h]   (視窗位置/大小，預設 0 0 480 360)
"""
import sys
from collections import deque

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

X, Y, W, H = [int(v) for v in sys.argv[1:5]] if len(sys.argv) >= 5 else (0, 0, 480, 360)
rclpy.init()
n = rclpy.create_node('demo_cam_view')
br = CvBridge()
st = {'img': None, 'ball': '', 'court': '', 'ev': deque(maxlen=3)}
n.create_subscription(Image, '/tennis_camera/image_raw', lambda m: st.update(img=m), qos_profile_sensor_data)


def uart(m):
    s = m.data.strip()
    if s.startswith('BALL') or s.startswith('NOBALL'):
        st['ball'] = s
    elif s.startswith('COURT'):
        st['court'] = s


n.create_subscription(String, '/jetson_uart', uart, 50)
n.create_subscription(String, '/stm32_event', lambda m: st['ev'].append(m.data.split(',', 1)[1]), 50)
win = '車上鏡頭 (D 程式 SIL)'
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.resizeWindow(win, W, H)
cv2.moveWindow(win, X, Y)
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.02)
    if st['img'] is None:
        cv2.waitKey(10)
        continue
    if not st.get('shown'):
        print('FIRST_IMAGE', flush=True)
        st['shown'] = True
    f = br.imgmsg_to_cv2(st['img'], 'bgr8').copy()
    edge = 'EDGE' in st['court']
    lines = [(st['ball'] or 'BALL: --', (0, 255, 255)), (st['court'] or 'COURT: --', (0, 0, 255) if edge else (0, 255, 0))]
    lines += [('STM32 ' + e.replace(',', ' '), (255, 255, 255)) for e in st['ev']]
    for i, (t, c) in enumerate(lines):
        cv2.putText(f, t, (8, 26 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(f, t, (8, 26 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2)
    cv2.imshow(win, f)
    if cv2.waitKey(10) == 27:
        break
cv2.destroyAllWindows()
rclpy.shutdown()
