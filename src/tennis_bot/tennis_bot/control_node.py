import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(Point, '/target_position', self.target_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.kp = 0.005
        self.max_omega = 0.6  # 降到 0.6 讓轉向更平滑，減少急煞偏航
        
        # 狀態機變數
        self.state = 'SEARCH' # 初始狀態：SEARCH, TRACK, BLIND_CAPTURE
        self.last_state_change_time = self.get_clock().now().nanoseconds / 1e9
        
        # 盲抓設定參數
        self.blind_duration = 0.8 # 盲抓強制直行時間 (秒)
        self.blind_speed = 0.3   # 盲抓時的推進速度 (稍微比最低速 0.1 快一點，確保能把球塞進去)

        self.get_logger().info('Control Node 最終版(平滑減速+盲抓接管)已啟動.')

    def target_callback(self, msg):
        twist = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9

        # ==========================================
        # 狀態 1：盲抓接管 (最優先級別，絕對不聽相機的話)
        # ==========================================
        if self.state == 'BLIND_CAPTURE':
            elapsed_blind = current_time - self.last_state_change_time
            if elapsed_blind < self.blind_duration:
                # 鎖死方向盤，強制直行
                twist.linear.x = self.blind_speed
                twist.angular.z = 0.0
                self.get_logger().info(f'!!! BLIND CAPTURE !!! - {elapsed_blind:.2f}s / {self.blind_duration}s')
                self.publisher_.publish(twist)
                return  # 直接 return，略過底下所有的相機邏輯
            else:
                # 盲抓時間結束，強制進入全域搜尋找下一顆球
                self.state = 'SEARCH'
                self.last_state_change_time = current_time

        # ==========================================
        # 狀態 2：視覺追蹤與平滑減速
        # ==========================================
        if msg.z == 1.0:
            if self.state != 'TRACK':
                self.state = 'TRACK'
                
            error_x = 320.0 - msg.x
            omega = error_x * self.kp
            
            # 轉向飽和限制
            if omega > self.max_omega: omega = self.max_omega
            elif omega < -self.max_omega: omega = -self.max_omega
            
            twist.angular.z = omega
            
            # 平滑減速邏輯
            if msg.y > 300:
                speed = 0.4 - ((msg.y - 300) * 0.002)
                
                # 【核心邏輯】：當速度降到 0.1 時，觸發盲抓接管！
                if speed <= 0.3:
                    self.state = 'BLIND_CAPTURE'
                    self.last_state_change_time = current_time
                    twist.linear.x = self.blind_speed
                    twist.angular.z = 0.0
                    self.get_logger().info('>>> Triggering BLIND CAPTURE <<<')
                    self.publisher_.publish(twist)
                    return
                else:
                    twist.linear.x = float(speed)
            else:
                twist.linear.x = 0.4 # 遠距離保持全速
                
            self.get_logger().info(f'Tracking - Err: {error_x:.1f}, Om: {omega:.2f}, V: {twist.linear.x:.2f}')
            
        # ==========================================
        # 狀態 3：全域搜尋 (遺失目標)
        # ==========================================
        else:
            elapsed_search = current_time - self.last_state_change_time
            
            # 如果剛剛還在追球，突然不見了，重置搜尋計時器
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

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControlNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()