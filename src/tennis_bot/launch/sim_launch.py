import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tennis_bot')
    
    # 1. 指定你的 URDF 與 World 檔案路徑
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_bot.urdf') 
    world_file = os.path.join(pkg_share, 'worlds', 'tennis_court.world') # 新增這行
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 自動抓取安裝路徑，注入 Gazebo 環境變數
    install_dir = os.path.join(pkg_share, '..')
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = install_dir if not gazebo_model_path else f"{gazebo_model_path}:{install_dir}"

    return LaunchDescription([
        # 1. 設定環境變數
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=new_model_path),
        
        # 2. 啟動 Gazebo，並在指令最後面加上 world_file 路徑
        # libgazebo_ros_init.so 負責發布 /clock，沒有它 use_sim_time 會卡在 0，
        # AMCL / RViz2 的 TF 對不上時間軸就會斷鏈
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),

        # 3. 啟動 Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),
        
        # 4. 啟動 Spawn Entity (負責把機器人放進剛開好的網球場裡)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            # 世界原點 x=0 是網子，車不能生在網子裡 (會被彈飛)，先放在我方半場中間，
            # grid_patrol_node 開始時會再傳送到格 1
            arguments=['-topic', 'robot_description', '-entity', 'tennis_bot', '-x', '-6.0', '-y', '0.0'],
            output='screen'
        )
    ])