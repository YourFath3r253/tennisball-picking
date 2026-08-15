"""每次要開始新的一輪巡邏之前先跑這個：把車體傳送到格 1 的座標 (面向 +x)，
球則維持世界檔案裡目前的位置不動 (如果要全新的隨機佈局，重開 Gazebo 讓
world 檔案重新載入球的初始位置就好，這個腳本只負責喬車體位置)。
"""
import sys
import rclpy
from gazebo_msgs.srv import SetEntityState

sys.path.insert(0, '/home/sean/ros2_ws/src/tennis_bot/tennis_bot')
from grid_waypoints import generate_grid_waypoints  # noqa: E402

X_RANGE = (-11.0, 11.0)
Y_RANGE = (-4.5, 4.5)
GRID_COLS = 8
GRID_ROWS = 4


def main():
    waypoints, cell_numbers, _, _ = generate_grid_waypoints(X_RANGE, Y_RANGE, GRID_COLS, GRID_ROWS)
    start_x, start_y = waypoints[0]
    print(f'格 1 座標: ({start_x:.3f}, {start_y:.3f})')

    rclpy.init()
    node = rclpy.create_node('reset_run')
    cli = node.create_client(SetEntityState, '/set_entity_state')
    if not cli.wait_for_service(timeout_sec=10.0):
        print('ERROR: /set_entity_state 服務沒有回應，Gazebo 有在跑嗎？')
        rclpy.shutdown()
        return

    req = SetEntityState.Request()
    req.state.name = 'tennis_bot'
    req.state.pose.position.x = start_x
    req.state.pose.position.y = start_y
    req.state.pose.position.z = 0.05
    req.state.pose.orientation.w = 1.0
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()
    print('車體傳送到格 1:', result.success if result else False)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
