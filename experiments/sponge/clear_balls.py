"""把 ball_2..ball_10 移到場外 (撿球單項測試只用 ball_1)。Gazebo 要開著。"""
import math
import rclpy
from gazebo_msgs.srv import SetEntityState

rclpy.init()
n = rclpy.create_node('clear_balls')
cli = n.create_client(SetEntityState, '/set_entity_state')
cli.wait_for_service()
for i in range(2, 11):
    r = SetEntityState.Request()
    r.state.name = f'ball_{i}'
    r.state.pose.position.x, r.state.pose.position.y, r.state.pose.position.z = 30.0 + 0.3 * i, 30.0, 0.033
    r.state.pose.orientation.w = 1.0
    r.state.reference_frame = 'world'
    f = cli.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
print('balls 2-10 cleared')
rclpy.shutdown()
