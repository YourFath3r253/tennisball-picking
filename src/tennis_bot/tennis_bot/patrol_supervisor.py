import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tennis_bot.coverage_planner import generate_boustrophedon_waypoints

# 對應 tennis_bot_maps/maps/generate_map.py 的球場尺寸與 1.3 倍留白設定
COURT_LENGTH_M = 24.0
COURT_WIDTH_M = 11.0
MARGIN_RATIO = 1.3

# 對應 simple_bot.urdf 的相機水平視角；detection_range_m 是暫定值，
# 等視覺那邊給出實際「有效辨識距離」後要換成真實數字
CAMERA_HFOV_RAD = 1.396
DETECTION_RANGE_M = 2.0
OVERLAP_RATIO = 0.2

# 追球/盲抓安全網：追太久沒結果（例如球在移動中一直沒撿到）就強制回去巡邏，
# 不要讓一顆撿不到的球卡死整場巡邏
CAPTURE_TIMEOUT_SEC = 30.0


def to_pose_stamped(clock, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = clock.now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class PatrolSupervisor(Node):
    """巡邏中若相機看到球就中斷 Nav2、把控制權交給 control_node 追蹤/盲抓，
    盲抓結束後再把控制權交還 Nav2，從剛剛中斷的航點繼續巡邏。
    對應 6/22, 5/21 簡報講的「動態任務切換」。
    """

    def __init__(self, navigator, goal_poses):
        super().__init__('patrol_supervisor')
        self.navigator = navigator
        self.goal_poses = goal_poses
        self.current_index = 0

        self.state = 'PATROL'  # PATROL 或 CAPTURE
        self.seen_blind_capture = False
        self.capture_start_time = None

        self.capture_mode_pub = self.create_publisher(Bool, '/capture_mode', 10)
        self.create_subscription(Point, '/target_position', self.target_callback, 10)
        self.create_subscription(String, '/control_node/state', self.control_state_callback, 10)

        self.get_logger().info(f'巡邏監督啟動，共 {len(goal_poses)} 個航點')
        self._send_current_waypoint()

    def _send_current_waypoint(self):
        if self.current_index >= len(self.goal_poses):
            self.get_logger().info('全場巡邏完成')
            return
        pose = self.goal_poses[self.current_index]
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.goToPose(pose)

    def _set_capture_mode(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.capture_mode_pub.publish(msg)

    def target_callback(self, msg):
        if self.state == 'PATROL' and msg.z == 1.0:
            self.get_logger().info('偵測到網球，中斷巡邏 -> 切換到追蹤/盲抓')
            self.navigator.cancelTask()
            self.state = 'CAPTURE'
            self.seen_blind_capture = False
            self.capture_start_time = self.get_clock().now().nanoseconds / 1e9
            self._set_capture_mode(True)

    def control_state_callback(self, msg):
        if self.state != 'CAPTURE':
            return
        if msg.data == 'BLIND_CAPTURE':
            self.seen_blind_capture = True
        elif self.seen_blind_capture:
            # 盲抓結束後 control_node 的狀態會離開 BLIND_CAPTURE (變 SEARCH 或 TRACK)，
            # 這就是「這顆球處理完了」的訊號
            self._finish_capture()

    def _finish_capture(self):
        self.get_logger().info('撿球流程結束 -> 恢復巡邏')
        self.state = 'PATROL'
        self._set_capture_mode(False)
        self._send_current_waypoint()

    def _check_capture_timeout(self):
        if self.state != 'CAPTURE' or self.capture_start_time is None:
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.capture_start_time
        if elapsed > CAPTURE_TIMEOUT_SEC:
            self.get_logger().warn(f'追球超過 {CAPTURE_TIMEOUT_SEC:.0f} 秒沒結果，強制恢復巡邏')
            self._finish_capture()

    def is_patrol_finished(self):
        return self.state == 'PATROL' and self.current_index >= len(self.goal_poses)

    def step(self):
        if self.state == 'PATROL' and self.current_index < len(self.goal_poses):
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'航點 {self.current_index + 1}/{len(self.goal_poses)} 完成')
                else:
                    self.get_logger().warn(f'航點 {self.current_index + 1} 未成功 ({result})，跳到下一個')
                self.current_index += 1
                self._send_current_waypoint()
        self._check_capture_timeout()


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    waypoints_xyyaw, lane_spacing = generate_boustrophedon_waypoints(
        court_length_m=COURT_LENGTH_M,
        court_width_m=COURT_WIDTH_M,
        margin_ratio=MARGIN_RATIO,
        camera_hfov_rad=CAMERA_HFOV_RAD,
        detection_range_m=DETECTION_RANGE_M,
        overlap_ratio=OVERLAP_RATIO,
    )
    navigator.get_logger().info(
        f'弓字型巡邏航點共 {len(waypoints_xyyaw)} 個，車道間距 {lane_spacing:.2f} m'
    )
    goal_poses = [to_pose_stamped(navigator.get_clock(), x, y, yaw) for x, y, yaw in waypoints_xyyaw]

    supervisor = PatrolSupervisor(navigator, goal_poses)

    try:
        while rclpy.ok() and not supervisor.is_patrol_finished():
            rclpy.spin_once(supervisor, timeout_sec=0.2)
            supervisor.step()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
