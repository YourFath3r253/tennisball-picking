import csv
import os
import random
import time
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Empty, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math # 新增 math 用於計算歐幾里得距離

from tennis_bot.half_court import LINES as COURT_LINES

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
# 2026-10-05 相機視角 80° -> 66.6° (對應 D 同學實測)，焦距 381.4 -> 487.5 px，同一顆球看起來變大
# (487.5/381.4)^2 = 1.63 倍，門檻跟著放大，維持「3 m 才開始抓」：55 -> 90、15 -> 25。
ENTER_CONTOUR_AREA = 90   # 新抓目標的門檻 (≈ 3 m)
LOCKED_CONTOUR_AREA = 25  # 已鎖定目標，面積掉到這以下才算真的看丟

# ---- D 同學提案：四色邊界偵測 (半場世界 half_court_lines.world 才有色帶，全場世界永遠是 SAFE) ----
# 只看畫面下方「離相機 BOUNDARY_TRIGGER_DIST_M 以內的地面」那一塊 (ROI)。相機高 0.15 m、水平朝前、
# 焦距 381.4 px (640px / 80° 水平視角)，地面上前方 d 公尺的點落在第 240 + f*h/d 列：
# d=0.6 m -> 第 335 列，所以 ROI = 第 335~479 列。某個顏色在 ROI 裡超過 BOUNDARY_MIN_PIXELS 個像素
# = 那條邊界已經在 0.6 m 以內 -> EDGE。左右半邊像素比較多的那側 = 邊界在哪一側 (LEFT/RIGHT/CENTER)。
# 發布格式模仿 D 同學實體車程式的 COURT,<SAFE|EDGE|OUT>,<LEFT|CENTER|RIGHT|NONE>,<比例>，
# 多加顏色跟這張畫面的模擬時間戳：COURT,EDGE,LEFT,0.120,BLUE,123.456
CAM_HEIGHT_M = 0.15
# ---- 相機參數對應 D 同學的實測校正 (2026-10-05) ----
# D 同學程式 (realtime_camera_trt_distance_uart_nodisplay.py)：FOCAL_LENGTH_PX=390、image_cx=283，
# 都是 512x384 畫面的值；換成 640x480：焦距 487.5 px (水平視角 66.6°，URDF 同步改)，
# 「正前方」在第 353.75 行 (URDF 相機往左偏 3.96° 來重現這個偏移)。
CAM_HFOV_DEG = 2.0 * math.degrees(math.atan(256.0 / 390.0))   # 66.6°
FOCAL_PX = 320.0 / math.tan(math.radians(CAM_HFOV_DEG / 2.0))  # 487.5 px
CAM_CX = 320.0 + (283.0 - 256.0) * 640.0 / 512.0               # 353.75 px
# D 同學 6/21 靜態測試 (球放 50 cm 正中/左/右，實體車子程式碼/舊視覺數據_20260919分析)：
#   距離平均 47.0~48.0 cm (低估約 5.6%，標準差 0.24~0.52 cm)，角度標準差 0.27~0.89°。
# 模擬照這個加：距離乘 0.944 再加 1% 雜訊、角度加 0.6° 雜訊 (每一幀獨立)。
DIST_BIAS = 0.944
DIST_NOISE_FRAC = 0.01
BEARING_NOISE_DEG = 0.6
BOUNDARY_TRIGGER_DIST_M = 0.6
BOUNDARY_ROI_TOP = int(240 + FOCAL_PX * CAM_HEIGHT_M / BOUNDARY_TRIGGER_DIST_M)
BOUNDARY_MIN_PIXELS = 300
BOUNDARY_SIDE_RATIO = 1.5  # 一側像素是另一側的 1.5 倍以上才算偏那一側，不然 CENTER

# ---- 真實相機模型 (模擬真實化 R5)：更新率 + 延遲照實體車實測 ----
# 預設開啟；TENNISBOT_CAMERA_MODEL=ideal 可以關掉 (回到 Gazebo 30Hz 每幀立刻處理)。
# 更新率：實體車 run10/12/13/14/15 共約 2700 幀的 CSV，視覺輸出平均 4.4 Hz，而且間隔是固定的
#   三拍節奏 0.11 / 0.11 / 0.45 s (D 同學程式每 3 幀做一次球場邊界色彩分類，那一幀多花 ~0.34 s)。
# 延遲：run12 (Kp=1.5) 8 次左右擺盪，用「衝過頭的角度 ÷ 衝過零點時的角速度」估出整個迴路的
#   等效延遲 0.53~0.92 s (平均 0.70 s)，包含相機緩衝區舊幀 (OpenCV V4L2 預設 4 個 buffer，
#   處理比相機慢時讀到的是好幾拍以前的畫面)、TensorRT 推論、UART、馬達反應。
#   這裡的 FRAME_AGE 是「發布時用的那張畫面有多舊」，用 experiments/step_response_test.py
#   校正到模擬裡量出來的等效延遲跟實體車一樣 (見下面)。
CAMERA_MODEL = os.environ.get('TENNISBOT_CAMERA_MODEL', 'real')
# 2026-10-05 用 5 個 run、3199 個連續幀重新量：短間隔 0.1107±0.0037 s、長間隔 0.4525±0.0308 s，
# 嚴格「短短長」循環 (1053 次，只有 4 次長長、7 次短短短)，平均 4.45 Hz。
REAL_OUTPUT_INTERVALS = (0.111, 0.111, 0.453)
REAL_INTERVAL_JITTER = (0.004, 0.004, 0.031)   # 各自的標準差 (常態分布)
# 校正結果 (experiments/step_response_test.py，K=0.02 飽和 0.6 rad/s，用跟分析實體車一樣的方法)：
#   FRAME_AGE 0.40 -> 模擬等效延遲 0.55~0.56 s；0.45 -> 0.72 s；0.50 -> 0.79~0.81 s；實體車 0.70 s -> 取 0.45
#   (理想相機量出來是 0：沒有 overshoot)
# 2026-10-05 重新校正為 0.60：改成實體車的完整資料流之後 (D 同學程式的 EMA 平滑 + STM32 輪速 PID + 馬達模型，
#   見 experiments/實驗數據/motor_model_fit.md)，跟馬達參數一起用 Kp=0.3/0.6/1.5 三組實體車數據擬合出 0.60。
REAL_FRAME_AGE = float(os.environ.get('TENNISBOT_CAMERA_LATENCY', '0.60'))

# ---- 實體車的 UART 協定 (D 同學 realtime_camera_trt_distance_uart_nodisplay.py 原樣照做) ----
# 實體車：Jetson 每一幀算出最近那顆球的距離/角度，EMA 平滑 (alpha=0.35) 之後，連續偵測到 2 幀才開始送
#   "BALL,<距離cm>,<角度deg>"，之後每幀都送；連續 5 幀沒看到才送 "BALL_OFF"。角度：球在右邊為正。
# 模擬：同樣的字串發在 /jetson_uart (std_msgs/String)，控制節點只吃這個 topic (等於 STM32 收 UART)。
# 球場邊界 (D 同學提案) 也用 D 同學的格式 COURT,<SAFE|EDGE>,<LEFT|CENTER|RIGHT|NONE>,<比例>，後面多一欄顏色。
BALL_DIAMETER_CM = 6.7
UART_EMA_ALPHA = 0.35
BALL_ON_CONFIRM_FRAMES = 2
BALL_OFF_CONFIRM_FRAMES = 5

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(Image, '/tennis_camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Point, '/target_position', 10)
        self.debug_publisher = self.create_publisher(Image, '/vision/debug_image', 10)
        self.boundary_pub = self.create_publisher(String, '/court_boundary', 10)
        self.uart_pub = self.create_publisher(String, '/jetson_uart', 10)
        self._uart_smooth = None       # (dist_cm, bearing_deg) EMA
        self._detected_streak = 0
        self._missing_streak = 0
        self._motor_on = False
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

        # 真實相機模型用：最近 ~1.5 秒的畫面 (模擬時間戳, msg)，跟下一次輸出的模擬時間
        self._frame_buffer = deque()
        self._next_output_stamp = None
        self._output_count = 0
        self._rng = random.Random(0)

        mode = (f'真實相機模型 (輸出間隔 {REAL_OUTPUT_INTERVALS} s，畫面延遲 {REAL_FRAME_AGE:.2f} s)'
                if CAMERA_MODEL == 'real' else '理想相機 (30Hz、無延遲)')
        self.get_logger().info(f'Vision Node 升級版(目標鎖定)已啟動... {mode}')

    def _reset_lock_cb(self, msg):
        self.locked_cx, self.locked_cy = None, None

    def image_callback(self, msg):
        if CAMERA_MODEL != 'real':
            # 理想相機：Gazebo 每來一幀 (30Hz) 就立刻處理、立刻發布，沒有延遲
            self._process_frame(msg)
            return
        # 真實相機：畫面先存起來，到了輸出時間才拿「FRAME_AGE 秒以前」的那張來處理。
        # 全部用模擬時間 (影像時間戳)，即時率 <1 時延遲也不會被拉長。
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._frame_buffer.append((stamp, msg))
        while self._frame_buffer and stamp - self._frame_buffer[0][0] > REAL_FRAME_AGE + 1.0:
            self._frame_buffer.popleft()
        if self._next_output_stamp is None:
            self._next_output_stamp = stamp + REAL_FRAME_AGE
        if stamp < self._next_output_stamp:
            return
        target = stamp - REAL_FRAME_AGE
        chosen = self._frame_buffer[0][1]
        for s, m in self._frame_buffer:
            if s <= target:
                chosen = m
            else:
                break
        self._process_frame(chosen)
        interval = REAL_OUTPUT_INTERVALS[self._output_count % len(REAL_OUTPUT_INTERVALS)]
        self._output_count += 1
        jitter = REAL_INTERVAL_JITTER[(self._output_count - 1) % len(REAL_INTERVAL_JITTER)]
        self._next_output_stamp = stamp + max(0.03, interval + self._rng.gauss(0.0, jitter))

    def _process_frame(self, msg):
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
        self._send_ball_uart(ball_found, best_contour, cx)
        self._detect_boundary(hsv, cv_image, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

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

    def _send_ball_uart(self, ball_found, contour, cx):
        """照 D 同學程式送 BALL / BALL_OFF (距離用球的外框大小推算，跟 D 同學 estimate_ball_position 同一套)。"""
        if ball_found and contour is not None:
            _, _, w, h = cv2.boundingRect(contour)
            ball_px = math.sqrt(max(1.0, w * h))
            z_cm = BALL_DIAMETER_CM * FOCAL_PX / ball_px
            x_cm = (cx - CAM_CX) * z_cm / FOCAL_PX
            dist = math.hypot(x_cm, z_cm) * DIST_BIAS * (1.0 + self._rng.gauss(0.0, DIST_NOISE_FRAC))
            bearing = math.degrees(math.atan2(x_cm, z_cm)) + self._rng.gauss(0.0, BEARING_NOISE_DEG)
            if self._uart_smooth is None:
                self._uart_smooth = (dist, bearing)
            else:
                a = UART_EMA_ALPHA
                self._uart_smooth = (a * dist + (1 - a) * self._uart_smooth[0],
                                     a * bearing + (1 - a) * self._uart_smooth[1])
            self._detected_streak += 1
            self._missing_streak = 0
            if self._detected_streak >= BALL_ON_CONFIRM_FRAMES:
                self._motor_on = True
                self.uart_pub.publish(String(data='BALL,%.1f,%.1f' % self._uart_smooth))
        else:
            self._detected_streak = 0
            self._missing_streak += 1
            if self._motor_on and self._missing_streak >= BALL_OFF_CONFIRM_FRAMES:
                self.uart_pub.publish(String(data='BALL_OFF'))
                self._motor_on = False

    def _detect_boundary(self, hsv, cv_image, frame_stamp):
        roi = hsv[BOUNDARY_ROI_TOP:, :]
        roi_area = roi.shape[0] * roi.shape[1]
        best = None  # (像素數, 顏色, 左半像素, 右半像素)
        for color, d in COURT_LINES.items():
            mask = None
            for lo, hi in d['hsv']:
                m = cv2.inRange(roi, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            n = int(cv2.countNonZero(mask))
            if n >= BOUNDARY_MIN_PIXELS and (best is None or n > best[0]):
                left = int(cv2.countNonZero(mask[:, :int(CAM_CX)]))  # 以車頭正前方 (不是畫面中心) 分左右
                best = (n, color, left, n - left)
        if best is None:
            text = f'COURT,SAFE,NONE,0.000,NONE,{frame_stamp:.3f}'
        else:
            n, color, left, right = best
            if left > BOUNDARY_SIDE_RATIO * right:
                side = 'LEFT'
            elif right > BOUNDARY_SIDE_RATIO * left:
                side = 'RIGHT'
            else:
                side = 'CENTER'
            text = f'COURT,EDGE,{side},{n / roi_area:.3f},{color},{frame_stamp:.3f}'
            cv2.putText(cv_image, f'EDGE {color} {side}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.line(cv_image, (0, BOUNDARY_ROI_TOP), (639, BOUNDARY_ROI_TOP), (255, 255, 255), 1)
        self.boundary_pub.publish(String(data=text))
        # UART 版：D 同學的 COURT 格式 + 顏色 (沒有時間戳，實體車 STM32 也拿不到畫面時間)
        self.uart_pub.publish(String(data=','.join(text.split(',')[:5])))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()