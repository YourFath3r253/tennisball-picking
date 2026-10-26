#!/bin/bash
# 新車體 v2 跑實體車程式 (D 的 Jetson 程式 + B 的 main.c)：把佈局寫進室內球場 -> 產生 pickup_test.world
# (ITF 球 57.7 g、步長 0.5 ms、球接觸 kp 1e6，避免 3D 列印薄牆穿模) -> run_real_car.sh
# 用法: bash experiments/run_real_car_v2.sh <tag> <layout.json> [模擬秒數=900]
# 環境變數：V2_ROLLER_KP / V2_ROLLER_KD / V2_ROLLER_R_COL (滾輪)、ROLLER_W (滾輪指令，預設 42.5)、TENNISBOT_FW_TRACK
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
python3 experiments/gen_ball_layout.py --load "$2" --half-court --world src/tennis_bot/worlds/indoor_court.world > /dev/null
python3 experiments/gen_pickup_world.py --mass ${BALL_MASS:-0.0577} > /dev/null
python3 experiments/cad_v2_to_urdf.py > /dev/null
colcon build --packages-select tennis_bot > /dev/null 2>&1
TENNISBOT_ROBOT=v2 WORLD=pickup_test.world LAYOUT_JSON="$2" TENNISBOT_BOUNCE_TIMEOUT=${3:-900} RUN_TIMEOUT_SEC=${RUN_TIMEOUT_SEC:-4000} \
  bash experiments/run_real_car.sh "$1"
