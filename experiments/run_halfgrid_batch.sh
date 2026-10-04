#!/bin/bash
# 比較基準：同樣的半場佈局，用原本的弓字路徑 (只走我方半場 16 格，需要里程計)。
# 用法: bash experiments/run_halfgrid_batch.sh <tag> <layout_json1> [layout_json2 ...]
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
TAG=$1; shift
MANIFEST="experiments/實驗數據/layouts/batch_${TAG}_manifest.csv"
echo "layout,run_dir,result" > "$MANIFEST"
for layout in "$@"; do
    python3 experiments/gen_half_court_world.py > /dev/null
    python3 experiments/gen_ball_layout.py --mass ${BALL_MASS:-0.027} --load "$layout" \
        --world src/tennis_bot/worlds/half_court_lines.world > /dev/null
    colcon build --packages-select tennis_bot > /dev/null 2>&1
    source install/setup.bash
    name=$(basename "$layout" .json)
    out=$(TENNISBOT_HALF_ONLY=1 WORLD=half_court_lines.world NODE=grid_patrol_node LAYOUT_JSON="$layout" \
          bash experiments/run_once.sh "${TAG}_${name}" 2>&1)
    run_dir=$(echo "$out" | grep -o "RUN_ONCE_DONE run[0-9]*" | awk '{print $2}')
    result=$(echo "$out" | grep -o "結束([^)]*)：[^ ]*" | tail -1)
    echo "$layout,$run_dir,$result" >> "$MANIFEST"
    echo "$name $run_dir $result"
done
echo "HALFGRID_BATCH_DONE"
