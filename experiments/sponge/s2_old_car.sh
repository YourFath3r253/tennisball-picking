#!/bin/bash
# S2 舊車撿球驗證：舊車 (v1 simple_bot.urdf) + 正常網球 57.7 g + pickup_test.world (0.5 ms、球 kp 1e6)
# 用法: bash experiments/sponge/s2_old_car.sh <out_dir> "<W們>" [offsets m，逗號] [盲衝速度 m/s]
#   W = /roller_cmd angular.z；舊車滾輪 = 1.556*W rad/s (W 42.5 = 66 rad/s = 631 rpm；W 82 = 128 rad/s = 1218 rpm)
cd /home/sean/ros2_ws
OUT=$1; WS=$2; OFFS=${3:-0,0.03,-0.03}; V=${4:-0.41}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
python3 experiments/gen_pickup_world.py --mass ${BALL_MASS:-0.0577} > /dev/null
colcon build --packages-select tennis_bot > /dev/null 2>&1
TENNISBOT_ROBOT=v1 WORLD=pickup_test.world GUI=false bash experiments/sim_up.sh > /dev/null
source install/setup.bash
python3 experiments/sponge/clear_balls.py
for off in ${OFFS//,/ }; do
  BALL_Y_OFF=$off timeout 900 python3 experiments/roller_pickup/pickup_test.py "$OUT/W_off${off}.csv" "${WS// /,}" "$V" 0.8 2>&1 | grep "^trial"
done
echo S2_DONE
