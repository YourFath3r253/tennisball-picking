#!/bin/bash
# 新車 v2 鏡頭仰角掃描：每個仰角重產 URDF + 重開 Gazebo -> C1/C3 (view_test) -> C2 (edge_stop_test)
# 用法: bash experiments/camera_tilt/sweep.sh <out_dir> "<仰角們>" [angles] [seeds]
# 巡邏速度用 B 新車版 120 rpm (TENNISBOT_PATROL_RPM)；轉向/追球沿用模擬裡的 B 10/12 版移植
cd /home/sean/ros2_ws
OUT=$1; TILTS=$2; ANGLES=${3:-90,60,-60,30,-30}; SEEDS=${4:-1,2}
mkdir -p "$OUT"
export TENNISBOT_PATROL_RPM=${TENNISBOT_PATROL_RPM:-120}
for t in $TILTS; do
  V2_CAM_PITCH_DEG=$t WORLD=indoor_court.world bash experiments/v2_restart.sh > /dev/null
  source /opt/ros/humble/setup.bash; source install/setup.bash
  grep -o 'rpy="0 0\.[0-9]* 0\.0691"' src/tennis_bot/urdf/tennis_bot_v2.urdf | head -1
  timeout 900 python3 experiments/camera_tilt/view_test.py "$OUT/view_tilt$t" --tilt $t 2>&1 | tail -1
  timeout 3000 python3 experiments/camera_tilt/edge_stop_test.py "$OUT/edge" --tilt $t --angles $ANGLES --seeds $SEEDS 2>&1 \
    | grep -E "tilt|Error|error" | grep -v "^\["
done
bash experiments/sim_down.sh > /dev/null 2>&1
echo SWEEP_DONE
