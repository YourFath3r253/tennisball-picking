import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 請將這裡的絕對路徑換成你實際的使用者名稱 (假設為 user)
    map_yaml_file = '/home/sean/ros2_ws/src/tennis_bot_maps/maps/tennis_court_map.yaml'
    
    return LaunchDescription([
        # 1. 啟動 Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]
        ),
        # 2. 啟動 AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'alpha1': 0.2,
                'alpha2': 0.2,
                'alpha3': 0.2,
                'alpha4': 0.2,
                'max_particles': 2000,
                'min_particles': 500
            }]
        ),
        # 3. 啟動 Lifecycle Manager (Nav2 必備，用於激活上述節點)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )
    ])
