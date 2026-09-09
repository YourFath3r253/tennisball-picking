#!/bin/bash
# 自動跑14次模擬：1-10次 0.027kg 隨機9球佈局，11-14次 10kg 但沿用1-4次的佈局。
# 停止換下一輪的條件 (任一符合)：撿滿9顆 / 撞牆 / 走完32格弓字路徑。
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
source /home/sean/ros2_ws/install/setup.bash

SCRATCH=/tmp/claude-1000/-home-sean-ros2-ws/cfdbfabc-f4d0-4155-bb10-100d98577b2d/scratchpad
LAYOUT_DIR=$SCRATCH/layouts
mkdir -p "$LAYOUT_DIR"
MANIFEST=$SCRATCH/batch_manifest.csv
echo "batch_i,run_dir,mass,layout_file,stop_reason" > "$MANIFEST"

RUN_TIMEOUT_SEC=700

kill_all() {
    pkill -TERM -f "gzserver.*tennis_court.world" 2>/dev/null
    pkill -TERM -f "gzclient.*tennis_court.world" 2>/dev/null
    pkill -TERM -f "lib/tennis_bot/vision_node" 2>/dev/null
    pkill -TERM -f "lib/tennis_bot/grid_patrol_node" 2>/dev/null
    sleep 2
    pkill -KILL -f "gzserver.*tennis_court.world" 2>/dev/null
    pkill -KILL -f "gzclient.*tennis_court.world" 2>/dev/null
    sleep 1
}

latest_run_dir() {
    ls -d "/home/sean/ros2_ws/experiments/實驗數據"/run* 2>/dev/null | sed 's#.*/run##' | sort -n | tail -1
}

for i in $(seq 1 14); do
    if [ "$i" -le 10 ]; then
        mass=0.027
        layout_file="$LAYOUT_DIR/run${i}.json"
        python3 experiments/gen_ball_layout.py --mass "$mass" --seed "$i" --save "$layout_file" > "$SCRATCH/genball_${i}.log" 2>&1
    else
        src_i=$((i - 10))
        mass=10.0
        layout_file="$LAYOUT_DIR/run${src_i}.json"
        python3 experiments/gen_ball_layout.py --mass "$mass" --load "$layout_file" > "$SCRATCH/genball_${i}.log" 2>&1
    fi
    if [ $? -ne 0 ]; then
        echo "batch $i: gen_ball_layout 失敗，跳過" >> "$SCRATCH/batch_progress.log"
        continue
    fi

    colcon build --packages-select tennis_bot > "$SCRATCH/build_${i}.log" 2>&1
    kill_all

    before_n=$(latest_run_dir)
    before_n=${before_n:-0}

    nohup ros2 launch tennis_bot sim_launch.py > "$SCRATCH/gazebo_batch_${i}.log" 2>&1 &
    gz_waited=0
    while [ $gz_waited -lt 30 ]; do
        if grep -q "wheel transforms" "$SCRATCH/gazebo_batch_${i}.log" 2>/dev/null; then
            break
        fi
        sleep 2
        gz_waited=$((gz_waited + 2))
    done
    sleep 2
    nohup ros2 run tennis_bot vision_node > "$SCRATCH/vision_batch_${i}.log" 2>&1 &
    sleep 3
    TENNISBOT_STOP_ON_RECOVER=1 nohup ros2 run tennis_bot grid_patrol_node > "$SCRATCH/grid_batch_${i}.log" 2>&1 &

    echo "batch $i 開始: mass=$mass $(date '+%H:%M:%S')" >> "$SCRATCH/batch_progress.log"

    # 給 grid_patrol_node 一點時間真的啟動起來，避免下面的迴圈第一次檢查時
    # process 還沒 fork/exec 完成，被誤判成「已經結束」。
    node_up=0
    for _ in $(seq 1 10); do
        if pgrep -f "lib/tennis_bot/grid_patrol_node" > /dev/null 2>&1; then
            node_up=1
            break
        fi
        sleep 1
    done
    if [ "$node_up" -ne 1 ]; then
        echo "batch $i: grid_patrol_node 沒有成功啟動" >> "$SCRATCH/batch_progress.log"
        kill_all
        echo "$i,啟動失敗,$mass,$layout_file,啟動失敗" >> "$MANIFEST"
        continue
    fi

    waited=0
    while [ $waited -lt $RUN_TIMEOUT_SEC ]; do
        if grep -q "結束(" "$SCRATCH/grid_batch_${i}.log" 2>/dev/null; then
            break
        fi
        if ! pgrep -f "lib/tennis_bot/grid_patrol_node" > /dev/null 2>&1; then
            break
        fi
        sleep 5
        waited=$((waited + 5))
    done

    reason=$(grep -o "結束([^)]*)" "$SCRATCH/grid_batch_${i}.log" 2>/dev/null | tail -1)
    if [ -z "$reason" ]; then
        reason="逾時或異常"
    fi
    pkill -INT -f "lib/tennis_bot/grid_patrol_node" 2>/dev/null
    sleep 3

    after_n=$(latest_run_dir)
    after_n=${after_n:-0}
    if [ "$after_n" -gt "$before_n" ]; then
        run_dir="run${after_n}"
    else
        run_dir="unknown"
    fi

    echo "$i,$run_dir,$mass,$layout_file,$reason" >> "$MANIFEST"
    echo "batch $i 結束: run_dir=$run_dir reason=$reason $(date '+%H:%M:%S')" >> "$SCRATCH/batch_progress.log"

    kill_all
done

echo "ALL_BATCH_DONE" >> "$SCRATCH/batch_progress.log"
