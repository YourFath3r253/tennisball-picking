#!/bin/bash
# 重新產生 v2 URDF (可用環境變數 V2_*) -> colcon build -> 重開 Gazebo (室內球場，新車)
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
python3 experiments/cad_v2_to_urdf.py > /dev/null
colcon build --packages-select tennis_bot > /dev/null 2>&1
TENNISBOT_ROBOT=v2 WORLD=${WORLD:-indoor_court.world} GUI=false bash experiments/sim_up.sh > /dev/null 2>&1
echo "v2 restarted: $(grep -o 'radius="0.0[0-9]*"' src/tennis_bot/urdf/tennis_bot_v2.urdf | sort | uniq -c | tr '\n' ' ')"
