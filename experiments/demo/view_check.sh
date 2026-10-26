#!/bin/bash
# 開 v2 GUI (pickup_test.world 現有設定) -> 跟車 -> 抓一張螢幕 -> 叫回 Claude
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
printf '[geometry]\nx=480\ny=0\nwidth=1440\nheight=1050\n' > ~/.gazebo/gui.ini
TENNISBOT_ROBOT=v2 WORLD=pickup_test.world GUI=true bash experiments/sim_up.sh > /dev/null
sleep 3
timeout 10 bash experiments/demo/follow_cam.sh tennis_bot
DISPLAY=:1 python3 experiments/demo/winctl.py minimize Claude > /dev/null
DISPLAY=:1 python3 experiments/demo/winctl.py raise Gazebo > /dev/null
sleep 2
DISPLAY=:1 timeout 10 gst-launch-1.0 -q ximagesrc num-buffers=1 use-damage=false ! videoconvert ! videoscale ! \
  video/x-raw,width=960,height=540 ! jpegenc ! filesink location="$1"
DISPLAY=:1 python3 experiments/demo/winctl.py raise Claude > /dev/null
echo VIEW_CHECK_DONE
