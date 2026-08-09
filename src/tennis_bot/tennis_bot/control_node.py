import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(Point, '/target_position', self.target_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.roller_publisher_ = self.create_publisher(Twist, '/roller_cmd', 10)

        # patrol_supervisor 用這個開關把車的控制權從 Nav2 巡邏切到這裡（追球/盲抓），
        # 沒被啟用時完全不動作，避免跟 Nav2 的 velocity_smoother 搶著發 /cmd_vel
        self.enabled = False
        self.enable_subscription = self.create_subscription(Bool, '/capture_mode', self.capture_mode_callback, 10)
        self.state_pub = self.create_publisher(String, '/control_node/state', 10)

        self.kp = 0.005
        self.max_omega = 0.6  # 降到 0.6 讓轉向更平滑，減少急煞偏航

        # 對齊角度門檻：水平視角 80 度 (1.396 rad) / 640px 換算約 458.5 px/rad，
        # ±5 度 (0.0873 rad) 對應約 40px。進入 APPROACH 用較嚴格的門檻對齊，
        # 已經在 APPROACH 時用寬一點的門檻 (2x) 才退回 ALIGN，避免在邊界上抖動。
        self.align_threshold_px = 40.0
        self.realign_threshold_px = 80.0

        # 狀態機變數
        self.state = 'SEARCH' # 初始狀態：SEARCH, TRACK, BLIND_CAPTURE
        self.track_phase = 'ALIGN'  # TRACK 底下的子狀態：ALIGN (原地轉正) -> APPROACH (直走接近)
        self.last_state_change_time = self.get_clock().now().nanoseconds / 1e9
        self.last_seen_time = None  # 上次真的偵測到球的時間，用來做遺失目標的寬限期判斷

        # 盲抓設定參數 + 滾輪轉速：宣告成 ROS2 參數，方便撿球實驗掃參數時用
        # `ros2 param set` 動態調整，不用每組參數都重開節點
        self.blind_duration = 0.8 # 盲抓強制直行時間 (秒)，這次實驗先固定不掃
        self.declare_parameter('blind_speed', 0.3)   # 盲抓時的推進速度 (m/s)
        self.declare_parameter('roller_omega', 30.0) # 撿球滾輪角速度 (rad/s)，對應 /roller_cmd angular.z

        # 滾輪指令用固定頻率的 timer 持續發送（比照真實硬體：啟用期間滾輪就是常轉），
        # 而不是只在 target_callback 觸發時發一次
        self.roller_timer = self.create_timer(0.1, self._publish_roller_cmd)

        self.get_logger().info('Control Node 最終版(平滑減速+盲抓接管+可調滾輪)已啟動.')

    def _publish_roller_cmd(self):
        twist = Twist()
        if self.enabled:
            twist.angular.z = float(self.get_parameter('roller_omega').value)
        else:
            twist.angular.z = 0.0
        self.roller_publisher_.publish(twist)

    def capture_mode_callback(self, msg):
        newly_enabled = msg.data and not self.enabled
        self.enabled = msg.data
        if newly_enabled:
            # 每次被喚醒接管都從乾淨的 SEARCH 狀態開始，不要沿用上次殘留的狀態
            self.state = 'SEARCH'
            self.track_phase = 'ALIGN'
            self.last_state_change_time = self.get_clock().now().nanoseconds / 1e9
            self.last_seen_time = None

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def target_callback(self, msg):
        if not self.enabled:
            return

        twist = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9

        # ==========================================
        # 狀態 1：盲抓接管 (最優先級別，絕對不聽相機的話)
        # ==========================================
        if self.state == 'BLIND_CAPTURE':
            elapsed_blind = current_time - self.last_state_change_time
            if elapsed_blind < self.blind_duration:
                # 鎖死方向盤，強制直行
                twist.linear.x = float(self.get_parameter('blind_speed').value)
                twist.angular.z = 0.0
                self.get_logger().info(f'!!! BLIND CAPTURE !!! - {elapsed_blind:.2f}s / {self.blind_duration}s')
                self._publish_state()
                self.publisher_.publish(twist)
                return  # 直接 return，略過底下所有的相機邏輯
            else:
                # 盲抓時間結束，強制進入全域搜尋找下一顆球
                self.state = 'SEARCH'
                self.last_state_change_time = current_time

        # ==========================================
        # 狀態 2：視覺追蹤 - 先原地轉正對齊，再直走接近
        # ==========================================
        if msg.z == 1.0:
            self.last_seen_time = current_time
            if self.state != 'TRACK':
                self.state = 'TRACK'
                self.track_phase = 'ALIGN'  # 每次重新鎖定目標都先從對齊開始

            error_x = 320.0 - msg.x

            # APPROACH 時如果偏移角度變太大 (球偏移或車體漂移)，退回 ALIGN 重新轉正；
            # 用比對齊門檻寬的 realign_threshold_px，避免在門檻邊界上抖動
            if self.track_phase == 'APPROACH' and abs(error_x) > self.realign_threshold_px:
                self.track_phase = 'ALIGN'

            # ---- 子狀態 1：ALIGN，原地左右轉，不前進 ----
            if self.track_phase == 'ALIGN':
                if abs(error_x) > self.align_threshold_px:
                    omega = error_x * self.kp
                    if omega > self.max_omega: omega = self.max_omega
                    elif omega < -self.max_omega: omega = -self.max_omega
                    twist.angular.z = omega
                    twist.linear.x = 0.0
                    self.get_logger().info(f'Aligning - Err: {error_x:.1f}, Om: {omega:.2f}')
                else:
                    self.track_phase = 'APPROACH'  # 已經轉到 ±5 度內，換直走

            # ---- 子狀態 2：APPROACH，已經對準了，直走接近 (不再邊走邊轉) ----
            if self.track_phase == 'APPROACH':
                twist.angular.z = 0.0

                # 平滑減速邏輯
                if msg.y > 300:
                    speed = 0.4 - ((msg.y - 300) * 0.002)

                    # 【核心邏輯】：當速度降到 0.1 時，觸發盲抓接管！
                    if speed <= 0.3:
                        self.state = 'BLIND_CAPTURE'
                        self.last_state_change_time = current_time
                        twist.linear.x = float(self.get_parameter('blind_speed').value)
                        twist.angular.z = 0.0
                        self.get_logger().info('>>> Triggering BLIND CAPTURE <<<')
                        self._publish_state()
                        self.publisher_.publish(twist)
                        return
                    else:
                        twist.linear.x = float(speed)
                else:
                    twist.linear.x = 0.4 # 遠距離保持全速

                self.get_logger().info(f'Approaching - Err: {error_x:.1f}, V: {twist.linear.x:.2f}')
            
        # ==========================================
        # 狀態 3：全域搜尋 (遺失目標)
        # ==========================================
        else:
            # 簡單色域偵測本來就會單幀漏檢（同一顆球同一個距離，偵測率可能才 40-50%），
            # 如果一偵測不到就馬上跳回 SEARCH，TRACK 狀態永遠沒辦法累積、球會一直
            # 追不到。加一個短暫的「寬限期」：剛看丟目標的這一小段時間內維持原本動作
            # (沿用上一次的 twist)，真的連續看丟超過 lost_target_grace 秒才算真的遺失。
            lost_target_grace = 0.3
            time_since_seen = current_time - self.last_seen_time if self.last_seen_time is not None else float('inf')
            if self.state == 'TRACK' and time_since_seen < lost_target_grace:
                return  # 保持上一次發布的指令，不要因為單幀漏檢就整個重新開始搜尋

            elapsed_search = current_time - self.last_state_change_time

            # 如果剛剛還在追球，確定是真的遺失了 (超過寬限期)，重置搜尋計時器
            if self.state == 'TRACK':
                self.state = 'SEARCH'
                self.last_state_change_time = current_time
                elapsed_search = 0.0

            # 搜尋邏輯：自轉 12.5 秒 -> 直行 15 秒 
            # 這裡利用 Python 的 % 餘數運算，讓它能無限循環
            cycle_time = elapsed_search % 27.5 
            
            if cycle_time < 12.5:
                # 旋轉階段
                twist.linear.x = 0.0
                twist.angular.z = 0.5
                self.get_logger().info(f'Search (SPIN) - {cycle_time:.1f}s')
            else:
                # 直行探索階段
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                self.get_logger().info(f'Search (MOVE) - {cycle_time:.1f}s')

        self._publish_state()
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControlNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()