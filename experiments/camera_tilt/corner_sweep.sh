#!/bin/bash
# C4b 角落測試掃描：bash experiments/camera_tilt/corner_sweep.sh <out_dir> "<仰角們>"
cd /home/sean/ros2_ws
OUT=$1; TILTS=$2
export TENNISBOT_PATROL_RPM=${TENNISBOT_PATROL_RPM:-120}
for t in $TILTS; do
  V2_CAM_PITCH_DEG=$t WORLD=indoor_court.world bash experiments/v2_restart.sh > /dev/null
  source /opt/ros/humble/setup.bash; source install/setup.bash
  timeout 3000 python3 experiments/camera_tilt/corner_test.py "$OUT" --tilt $t 2>&1 | grep "^tilt"
done
bash experiments/sim_down.sh > /dev/null 2>&1
echo CORNER_DONE
