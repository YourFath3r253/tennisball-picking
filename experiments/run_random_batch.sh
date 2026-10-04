#!/bin/bash
# 連跑 N 次隨機 9 顆球佈局：每次 gen_ball_layout 產生新佈局 (存 json) → colcon build
# (world 檔案要重新安裝) → run_once.sh → 結果寫進 manifest。
# 用法: bash experiments/run_random_batch.sh <tag> <seed1> [seed2 ...]
#   例: bash experiments/run_random_batch.sh open 101 102 103
# 可選環境變數: BALL_ARGS 會直接傳給 gen_ball_layout.py (例如 --net-clearance 0.7)
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
TAG=$1; shift
LAYOUT_DIR="experiments/實驗數據/layouts"
MANIFEST="$LAYOUT_DIR/batch_${TAG}_manifest.csv"
mkdir -p "$LAYOUT_DIR"
echo "seed,run_dir,result,layout_json" > "$MANIFEST"
for seed in "$@"; do
    layout="$LAYOUT_DIR/${TAG}_seed${seed}.json"
    python3 experiments/gen_ball_layout.py --mass ${BALL_MASS:-0.0577} --seed "$seed" --save "$layout" $BALL_ARGS > /dev/null
    colcon build --packages-select tennis_bot > /dev/null 2>&1
    source install/setup.bash
    out=$(LAYOUT_JSON="$layout" bash experiments/run_once.sh "${TAG}_s${seed}" 2>&1)
    run_dir=$(echo "$out" | grep -o "RUN_ONCE_DONE run[0-9]*" | awk '{print $2}')
    result=$(echo "$out" | grep -o "結束([^)]*)：[^ ]*" | tail -1)
    echo "$seed,$run_dir,$result,$layout" >> "$MANIFEST"
    echo "seed=$seed $run_dir $result"
done
# 跑完把 world 檔案還原成標準佈局
python3 experiments/gen_ball_layout.py --mass ${BALL_MASS:-0.0577} --load "$LAYOUT_DIR/standard.json" > /dev/null
colcon build --packages-select tennis_bot > /dev/null 2>&1
echo "RANDOM_BATCH_DONE"
cat "$MANIFEST"
