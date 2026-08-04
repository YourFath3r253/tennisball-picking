import math

import rclpy
from geometry_msgs.msg import PoseStamped
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


def to_pose_stamped(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


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

    goal_poses = [to_pose_stamped(navigator, x, y, yaw) for x, y, yaw in waypoints_xyyaw]
    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            navigator.get_logger().info(
                f'巡邏中: 第 {feedback.current_waypoint + 1}/{len(goal_poses)} 個航點'
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('全場巡邏完成')
    else:
        navigator.get_logger().warn(f'巡邏未正常完成: {result}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
