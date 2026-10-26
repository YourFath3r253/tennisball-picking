#!/bin/bash
# 錄影用：Gazebo 視窗跟著車子 (視角參數用 world 檔的 <track_visual>，見 gen_pickup_world.py GUI_TRACK_XYZ)
# 注意：這個檔名/指令列不能出現 Gazebo GUI 的程式名稱，sim_down.sh 的 pkill 會連帶殺掉
source /opt/ros/humble/setup.bash
gz camera -c gzclient_camera -f "${1:-tennis_bot}"
