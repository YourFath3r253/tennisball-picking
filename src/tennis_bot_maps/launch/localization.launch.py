import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tennis_bot_maps')
    map_yaml_file = os.path.join(pkg_share, 'maps', 'tennis_court_map.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'localization.rviz')

    return LaunchDescription([
        # 1. 啟動 Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file, 'use_sim_time': True}]
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
                'min_particles': 500,
                # URDF 的根座標系是 base_link，沒有 base_footprint，
                # 不改的話 AMCL 會用預設值找不存在的 frame 導致雷射一直對不到 TF
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'scan_topic': 'scan'
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
        ),
        # 4. 啟動 RViz2，帶入預先設定好的 Map/LaserScan/RobotModel/TF 顯示設定
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        )
    ])
