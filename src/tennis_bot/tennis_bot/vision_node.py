import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math # 新增 math 用於計算歐幾里得距離

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(Image, '/tennis_camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Point, '/target_position', 10)
        self.debug_publisher = self.create_publisher(Image, '/vision/debug_image', 10)
        self.bridge = CvBridge()
        
        # 3-2 目標鎖定記憶變數
        self.locked_cx = None
        self.locked_cy = None
        
        self.get_logger().info('Vision Node 升級版(目標鎖定)已啟動...')

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
        # 約 381px，3m 外的球影像面積理論值只剩約 55px^2，原本 100px 的門檻在這個
        # 距離會直接濾掉；門檻降到 12px^2 讓有效偵測距離延伸到約 3m (對應現實硬體
        # 的偵測極限)，同時還是遠高於雜訊等級的一兩個像素。
        MIN_CONTOUR_AREA = 12
        valid_contours = []
        for cnt in contours:
            if cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
                valid_contours.append(cnt)

        if len(valid_contours) > 0:
            # 3-2: 目標鎖定邏輯 (Target Locking)
            if self.locked_cx is not None:
                # 尋找距離上一幀最近的球 (歐幾里得距離)
                min_dist = float('inf')
                for cnt in valid_contours:
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        temp_cx = float(M["m10"] / M["m00"])
                        temp_cy = float(M["m01"] / M["m00"])
                        dist = math.hypot(temp_cx - self.locked_cx, temp_cy - self.locked_cy)
                        if dist < min_dist:
                            min_dist = dist
                            best_contour = cnt
                
                # 如果最近的球距離上一幀超過 150 像素，判定為失去原目標，重新找最大的
                if min_dist > 150:
                    best_contour = max(valid_contours, key=cv2.contourArea)
            else:
                # 第一次看到球，直接找最大的
                best_contour = max(valid_contours, key=cv2.contourArea)

            # 計算最終選定目標的中心點
            M = cv2.moments(best_contour)
            if M["m00"] > 0:
                cx = float(M["m10"] / M["m00"])
                cy = float(M["m01"] / M["m00"])
                self.locked_cx, self.locked_cy = cx, cy # 更新鎖定記憶
                ball_found = True

        if ball_found:
            target_msg.x = cx
            target_msg.y = cy
            target_msg.z = 1.0 
            cv2.drawContours(cv_image, [best_contour], -1, (0, 255, 0), 2)
            cv2.circle(cv_image, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.line(cv_image, (320, 240), (int(cx), int(cy)), (255, 0, 0), 2)
        else:
            self.locked_cx, self.locked_cy = None, None # 遺失目標，清除記憶
            target_msg.x, target_msg.y, target_msg.z = -1.0, -1.0, 0.0

        self.publisher_.publish(target_msg)
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