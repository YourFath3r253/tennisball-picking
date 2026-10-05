#!/bin/bash
# D 同學提案 (半場四色邊界、碰到邊界轉固定角度) 的批次實驗：
# 每個隨機佈局 (seed) 都用同一組球位置，依序跑每個轉向角度，只有角度不同 (一次只改一個變數)。
# 用法: bash experiments/run_bounce_batch.sh <tag> "<角度們>" <seed1> [seed2 ...]
#   例: bash experiments/run_bounce_batch.sh bounce "90 110 135 180" 501 502 503
# 可選環境變數: BALL_COUNT (預設 10)、BALL_MASS、GUI、BOUNCE_TIMEOUT (模擬秒數，預設 900)
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
TAG=$1; ANGLES=$2; shift 2
LAYOUT_DIR="experiments/實驗數據/layouts"
MANIFEST="$LAYOUT_DIR/batch_${TAG}_manifest.csv"
mkdir -p "$LAYOUT_DIR"
echo "angle,seed,run_dir,result,layout_json" > "$MANIFEST"
python3 experiments/gen_half_court_world.py > /dev/null
for seed in "$@"; do
    layout="$LAYOUT_DIR/${TAG}_seed${seed}.json"
    python3 experiments/gen_ball_layout.py --mass ${BALL_MASS:-0.027} --seed "$seed" --count ${BALL_COUNT:-10} \
        --half-court --world src/tennis_bot/worlds/half_court_lines.world --save "$layout" > /dev/null
    colcon build --packages-select tennis_bot > /dev/null 2>&1
    source install/setup.bash
    for angle in $ANGLES; do
        out=$(TENNISBOT_BOUNCE_ANGLE_DEG=$angle TENNISBOT_BOUNCE_TIMEOUT=${BOUNCE_TIMEOUT:-900} \
              WORLD=half_court_lines.world NODE=boundary_bounce_node LAYOUT_JSON="$layout" RUN_TIMEOUT_SEC=1400 \
              bash experiments/run_once.sh "${TAG}_a${angle}_s${seed}" 2>&1)
        run_dir=$(echo "$out" | grep -o "RUN_ONCE_DONE run[0-9]*" | awk '{print $2}')
        result=$(echo "$out" | grep -o "結束([^)]*)：[^ ]*" | tail -1)
        echo "$angle,$seed,$run_dir,$result,$layout" >> "$MANIFEST"
        echo "angle=$angle seed=$seed $run_dir $result"
    done
done
echo "BOUNCE_BATCH_DONE"
cat "$MANIFEST"
