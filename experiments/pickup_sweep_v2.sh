#!/bin/bash
# 撿球道參數掃描：海綿剛性 kp x 滾輪指令 W (球 57.7 g、pickup_test.world)
# 用法: bash experiments/pickup_sweep_v2.sh <out_dir> "<kp們>" "<W們>" [offsets]
cd /home/sean/ros2_ws
OUT=$1; KPS=$2; WS=$3; OFFS=${4:-0,0.03,-0.03}
mkdir -p "$OUT"
for kp in $KPS; do
  V2_ROLLER_KP=$kp WORLD=pickup_test.world bash experiments/v2_restart.sh > /dev/null
  source /opt/ros/humble/setup.bash; source install/setup.bash
  for W in $WS; do
    timeout 600 python3 experiments/pickup_test_v2.py "$OUT/kp${kp}_W${W}.csv" --W $W --offsets $OFFS 2>&1 \
      | grep "ball_1" | sed "s/^/kp=$kp /; s/最後位置 (車體座標) //"
  done
done
echo SWEEP_DONE
