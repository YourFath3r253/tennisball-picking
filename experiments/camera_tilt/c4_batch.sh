#!/bin/bash
# C4：新車 v2 鏡頭仰角整場驗證。每個 (仰角, 佈局) 跑一次完整的 D 程式 SIL + B 邏輯 (巡邏 120 rpm)，
# 球進撿球口就算撿到並移走 (TENNISBOT_PICK_AT_MOUTH=1，跟滾輪能不能送進車廂拆開)。
# 用法: bash experiments/camera_tilt/c4_batch.sh "<仰角們>" "<seed們>" [模擬秒數=900]
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
TILTS=$1; SEEDS=${2:-501 502 503}; SECS=${3:-900}
for t in $TILTS; do
  for s in $SEEDS; do
    L="experiments/實驗數據/layouts/realcar_seed$s.json"
    python3 experiments/gen_ball_layout.py --load "$L" --half-court --mass 0.0577 \
      --world src/tennis_bot/worlds/indoor_court.world > /dev/null
    V2_CAM_PITCH_DEG=$t python3 experiments/cad_v2_to_urdf.py > /dev/null
    colcon build --packages-select tennis_bot > /dev/null 2>&1
    TENNISBOT_ROBOT=v2 WORLD=indoor_court.world LAYOUT_JSON="$L" TENNISBOT_BOUNCE_TIMEOUT=$SECS \
      TENNISBOT_PICK_AT_MOUTH=1 TENNISBOT_PATROL_RPM=120 RUN_TIMEOUT_SEC=${RUN_TIMEOUT_SEC:-3000} \
      bash experiments/run_real_car.sh "c4_tilt${t}_s$s" | tail -1 | sed "s/^/tilt=$t seed=$s /"
    d=$(ls -d experiments/實驗數據/run* | sed 's#.*/run##' | sort -n | tail -1)
    echo "cam_pitch_deg=$t" >> "experiments/實驗數據/run$d/meta.txt"
  done
done
echo C4_DONE
