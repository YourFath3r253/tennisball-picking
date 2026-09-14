#!/bin/bash
# 實驗數據收集：開放場地+網子，隨機 10/15/20 顆球各跑 5 次 (共 15 次)。
# 用法: bash experiments/run_data_batch.sh
# 每個球數一個 manifest: experiments/實驗數據/layouts/batch_data<N>_manifest.csv
cd /home/sean/ros2_ws
for count in 10 15 20; do
    seeds=""
    for i in 1 2 3 4 5; do seeds="$seeds $((count * 100 + i))"; done
    BALL_ARGS="--count $count --net-clearance 1.0" bash experiments/run_random_batch.sh "data$count" $seeds
done
echo "DATA_BATCH_ALL_DONE"
