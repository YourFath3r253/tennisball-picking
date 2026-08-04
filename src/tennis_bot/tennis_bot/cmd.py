#Terminal 1 (啟動 Gazebo 與物理環境):
'''
cd ~/ros2_ws
colcon build --packages-select tennis_bot
source install/setup.bash
ros2 launch tennis_bot sim_launch.py
'''

# 2 強制啟動 Robot State Publisher
# ros2 run robot_state_publisher robot_state_publisher /home/sean/ros2_ws/src/tennis_bot/urdf/simple_bot.urdf

# 3 啟動 AMCL 與地圖 (現在 tennis_bot_maps 是正式 package，RViz2 也內建在裡面一起啟動了)
# colcon build --packages-select tennis_bot_maps && source install/setup.bash
# ros2 launch tennis_bot_maps localization.launch.py

# 前後旋轉按鍵  si (前進)、, (後退)、j (左轉)、l (右轉)、k (停止)
'''
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
'''

# 輸入你選定的滾輪角速度 (以參數組 2 為例，20 rad/s)：
# 20~50 rad/s -> choose 30 rad/s
'''
ros2 topic pub /roller_cmd geometry_msgs/msg/Twist "{angular: {z: 47.50}}"
'''

#Terminal 2 (啟動視覺感測節點):
'''
cd ~/ros2_ws
source install/setup.bash
ros2 run tennis_bot vision_node
'''

#Terminal 3 (啟動大腦控制節點):
"""
cd ~/ros2_ws
source install/setup.bash
ros2 run tennis_bot control_node
"""


# delete old gazebo
'''
killall -9 gzserver gzclient
'''

#ball6 <pose>0.125 0.5 0.1 0 0 0</pose>

# rviz2
# ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

# ros2 launch ~/ros2_ws/src/tennis_bot_maps/launch/localization.launch.py


'''
第一階段：設定固定座標系 (Global Options)
Fixed Frame：必須填入 map。
工程觀念： 絕對不要改成 lidar_link 或 base_link。全域導航的上帝視角必須是 map。此時出現你截圖中的 Frame [map] does not exist 紅字完全正常，因為 AMCL 正在等待你給予初始位置，尚未廣播 map 座標系。
第二階段：載入視覺化模組 (Add Displays)
載入地圖 (Map)：
點擊左下角 Add $\rightarrow$ By topic $\rightarrow$ 選擇 /map (Map)。
展開 Map 設定，將 Durability Policy 改為 Transient Local。
此時畫面應出現 1:1 的網球場黑白地圖。
載入光達點雲 (LaserScan)：
點擊 Add $\rightarrow$ By topic $\rightarrow$ 選擇 /scan (LaserScan)。
將 Size (m) 設為 0.05。
將 Color Transformer 設為 FlatColor，並將顏色調為鮮紅色 (255; 0; 0)。
注意：此時你還看不到紅色點點，因為座標系尚未連線。
載入車體模型 (RobotModel)：
點擊 Add $\rightarrow$ By display type $\rightarrow$ 選擇 RobotModel。
設定 Description Topic 為 /robot_description。
這能讓你在 RViz2 中直接看到你的實體車輛外觀，是 Debug 的關鍵。
第三階段：座標系閉環與喚醒 (Initialization)
給定初始位置 (2D Pose Estimate)：
觀察 Gazebo 中車子的實際位置與朝向。
點擊 RViz2 上方工具列的綠色箭頭 2D Pose Estimate。
在 RViz2 地圖上與 Gazebo 對應的位置，按住滑鼠左鍵，並順著車頭方向拖曳出一個箭頭後放開。

'''