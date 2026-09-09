import csv
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import numpy as np
import math # 新增 math 用於計算歐幾里得距離

# 除錯用：記錄每一幀的輪廓偵測狀況 (數量/面積)，用來確認漏球是不是視覺這一層造成的。
# 固定路徑、每次啟動節點時覆寫，不用跟 grid_patrol_node 的 runN 資料夾綁在一起，
# 事後用兩邊各自的時間戳 (epoch) 對齊即可。
DEBUG_LOG_PATH = '/home/sean/ros2_ws/experiments/實驗數據/vision_debug.csv'
DEBUG_LOG_MIN_INTERVAL_SEC = 0.1

# 目標鎖定用：鎖定期間只判斷「這一幀有沒有候選夠接近、算同一顆球」，不跟其他
# 候選比較大小/距離 (不會因為別顆球比較大/比較近就搶走鎖定)。沒有夠接近的候選
# 就立刻放開鎖定，允許重新抓。
SAME_BALL_PX = 150

# 雙門檻 (hysteresis)：球剛好停在偵測距離邊緣時，輪廓面積會在門檻附近抖動，
# 用同一個門檻判斷「有效/看丟」會一幀有一幀沒有，反覆觸發對準/放棄。改成
# 「新抓目標」用嚴格門檻 55px^2 (對應理論3m)，但「已經鎖定的球，只要沒掉到
# 40px^2 以下都還算同一顆」，中間留緩衝帶，減少單純因為面積抖動造成的閃爍。
# 這個做法在真實相機上也適用 (YOLOv8信心值分數用同樣的雙門檻邏輯一樣成立)，
# 不是只有模擬用的招。
ENTER_CONTOUR_AREA = 55   # 新抓目標的門檻 (跟原本 MIN_CONTOUR_AREA 一樣)
LOCKED_CONTOUR_AREA = 15  # 已鎖定目標，面積掉到這以下才算真的看丟

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(Image, '/tennis_camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Point, '/target_position', 10)
        self.debug_publisher = self.create_publisher(Image, '/vision/debug_image', 10)
        # grid_patrol_node 在盲衝(BLIND_DASH)結束時會發這個，收到才放開鎖定，
        # 讓「鎖定同一顆球，追到盲衝結束才換目標」這件事由狀態機那邊決定時機，
        # 不是 vision_node 自己每幀憑距離判斷。
        self.create_subscription(Empty, '/vision_reset_lock', self._reset_lock_cb, 10)
        self.bridge = CvBridge()

        # 3-2 目標鎖定記憶變數
        self.locked_cx = None
        self.locked_cy = None

        self._debug_log_file = open(DEBUG_LOG_PATH, 'w', newline='')
        self._debug_log_writer = csv.writer(self._debug_log_file)
        self._debug_log_writer.writerow([
            'epoch', 'all_contour_count', 'all_max_area',
            'valid_contour_count', 'valid_max_area', 'ball_found', 'cx', 'cy',
        ])
        self._last_debug_log_time = 0.0

        self.get_logger().info('Vision Node 升級版(目標鎖定)已啟動...')

    def _reset_lock_cb(self, msg):
        self.locked_cx, self.locked_cy = None, None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # 遠處的球在畫面上只有幾個像素，容易因為單一像素的雜訊斷裂成好幾塊碎片、
        # 各自都小於面積門檻而被濾掉。做一次形態學閉運算 (先膨脹再侵蝕) 把鄰近的
        # 碎片黏合成一塊完整輪廓，遠距離偵測才不會整團漏掉。
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        target_msg = Point()
        ball_found = False
        cx, cy = -1.0, -1.0
        best_contour = None

        # 篩選有效輪廓：網球實際直徑 6.6cm、相機水平視角 80 度、640px 寬換算焦距
        # 約 381px，理論上剛好 3m 處面積是 55px^2 (面積門檻=理論上限距離的判定)。
        # 之前門檻是 12px^2 (理論上會偵測到更遠，但實測因為抗鋸齒邊緣像素被稀釋，
        # 實際只穩定到 1.3~1.5m)。改成 55px^2，讓「判定成球」直接對應理論3m，
        # 不再放寬到理論範圍以外。
        valid_contours = []       # 嚴格門檻 (55px^2)：抓新目標用
        locked_ok_contours = []   # 寬鬆門檻 (40px^2)：判斷已鎖定的球還在不在用
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > ENTER_CONTOUR_AREA:
                valid_contours.append(cnt)
            if area > LOCKED_CONTOUR_AREA:
                locked_ok_contours.append(cnt)

        found_same_ball = False
        if self.locked_cx is not None and len(locked_ok_contours) > 0:
            # 只問「這一幀有沒有候選夠接近，算同一顆球在移動」，不跟其他候選比較，
            # 不會因為別顆球比較大/比較近就搶走鎖定。這裡用寬鬆門檻 (40px^2)，
            # 讓已經鎖定的球在面積抖動到55px^2以下時還能撐住，不會馬上被判定看丟。
            min_dist = float('inf')
            nearest_cnt = None
            for cnt in locked_ok_contours:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    temp_cx = float(M["m10"] / M["m00"])
                    temp_cy = float(M["m01"] / M["m00"])
                    dist = math.hypot(temp_cx - self.locked_cx, temp_cy - self.locked_cy)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_cnt = cnt
            if min_dist <= SAME_BALL_PX:
                best_contour = nearest_cnt
                found_same_ball = True
        elif self.locked_cx is None and len(valid_contours) > 0:
            # 還沒有鎖定任何東西 (剛開始，或剛被放開)，直接找最大的 (嚴格門檻)
            best_contour = max(valid_contours, key=cv2.contourArea)
            found_same_ball = True

        if self.locked_cx is not None and not found_same_ball:
            # 這一幀連寬鬆門檻的候選都沒有，才真的判定看丟，立刻放開鎖定
            # (不再等待/凍結)，有新候選 (嚴格門檻) 的話馬上重新抓
            self.locked_cx, self.locked_cy = None, None
            if len(valid_contours) > 0:
                best_contour = max(valid_contours, key=cv2.contourArea)
                found_same_ball = True

        if found_same_ball and best_contour is not None:
            M = cv2.moments(best_contour)
            if M["m00"] > 0:
                cx = float(M["m10"] / M["m00"])
                cy = float(M["m01"] / M["m00"])
                self.locked_cx, self.locked_cy = cx, cy # 更新鎖定記憶
                ball_found = True
        elif self.locked_cx is not None:
            # 凍結：鎖定還在，但這一幀沒對到，沿用原本鎖定位置，不切換到別的候選
            cx, cy = self.locked_cx, self.locked_cy
            ball_found = True

        if ball_found:
            target_msg.x = cx
            target_msg.y = cy
            target_msg.z = 1.0
            if best_contour is not None:
                cv2.drawContours(cv_image, [best_contour], -1, (0, 255, 0), 2)
            cv2.circle(cv_image, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.line(cv_image, (320, 240), (int(cx), int(cy)), (255, 0, 0), 2)
        else:
            # 這一幀沒看到球，但鎖定記憶不清空——鎖定要一直咬住同一顆球，直到
            # grid_patrol_node 送出 /vision_reset_lock (盲衝結束) 才放開，不然
            # 每次暫時看不到就重置，等於又回到「每幀重新比大小」的舊行為。
            target_msg.x, target_msg.y, target_msg.z = -1.0, -1.0, 0.0

        self.publisher_.publish(target_msg)

        now = time.time()
        if now - self._last_debug_log_time >= DEBUG_LOG_MIN_INTERVAL_SEC:
            self._last_debug_log_time = now
            all_max_area = max((cv2.contourArea(c) for c in contours), default=0.0)
            valid_max_area = max((cv2.contourArea(c) for c in valid_contours), default=0.0)
            self._debug_log_writer.writerow([
                f'{now:.3f}', len(contours), f'{all_max_area:.1f}',
                len(valid_contours), f'{valid_max_area:.1f}',
                int(ball_found), f'{cx:.1f}', f'{cy:.1f}',
            ])
            self._debug_log_file.flush()

        try:
            self.debug_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except Exception:
            pass
        cv2.imshow("Robot Camera View", cv_image) # 顯示畫好準星的彩色畫面
        #cv2.imshow("HSV Mask", mask)              # 顯示到底什麼東西被判定成黃色 (全黑中帶有白塊)
        cv2.waitKey(1)                            # 這是 OpenCV 更新畫面的關鍵，一定要加！

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()