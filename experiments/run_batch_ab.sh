#!/bin/bash
# A/B測試：4組不同球佈局(0.027kg固定)，每組各跑一次「無輪速防呆」跟「有輪速防呆」，共8次。
# 撞牆判定已經改成用真實座標 (離牆0.2公尺內)，跟里程計無關，可以信任這個標籤。
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
source /home/sean/ros2_ws/install/setup.bash

SCRATCH=/tmp/claude-1000/-home-sean-ros2-ws/cfdbfabc-f4d0-4155-bb10-100d98577b2d/scratchpad
LAYOUT_DIR=$SCRATCH/layouts_ab
mkdir -p "$LAYOUT_DIR"
MANIFEST=$SCRATCH/batch_ab_manifest.csv
echo "batch_i,run_dir,layout,fix_enabled,layout_file,stop_reason" > "$MANIFEST"

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

i=0
for layout_n in 1 2 3 4; do
    layout_file="$LAYOUT_DIR/layout${layout_n}.json"
    python3 experiments/gen_ball_layout.py --mass 0.027 --seed $((200 + layout_n)) --save "$layout_file" > "$SCRATCH/genball_ab_${layout_n}.log" 2>&1

    for fix in 0 1; do
        i=$((i + 1))
        colcon build --packages-select tennis_bot > "$SCRATCH/build_ab_${i}.log" 2>&1
        kill_all

        before_n=$(latest_run_dir)
        before_n=${before_n:-0}

        nohup ros2 launch tennis_bot sim_launch.py > "$SCRATCH/gazebo_ab_${i}.log" 2>&1 &
        gz_waited=0
        while [ $gz_waited -lt 30 ]; do
            if grep -q "wheel transforms" "$SCRATCH/gazebo_ab_${i}.log" 2>/dev/null; then
                break
            fi
            sleep 2
            gz_waited=$((gz_waited + 2))
        done
        sleep 2
        nohup ros2 run tennis_bot vision_node > "$SCRATCH/vision_ab_${i}.log" 2>&1 &
        sleep 3
        TENNISBOT_STOP_ON_RECOVER=1 TENNISBOT_STALE_WHEEL_FIX=$fix \
            nohup ros2 run tennis_bot grid_patrol_node > "$SCRATCH/grid_ab_${i}.log" 2>&1 &

        echo "batch $i 開始: layout=$layout_n fix=$fix $(date '+%H:%M:%S')" >> "$SCRATCH/batch_ab_progress.log"

        node_up=0
        for _ in $(seq 1 10); do
            if pgrep -f "lib/tennis_bot/grid_patrol_node" > /dev/null 2>&1; then
                node_up=1
                break
            fi
            sleep 1
        done
        if [ "$node_up" -ne 1 ]; then
            echo "batch $i: grid_patrol_node 沒有成功啟動" >> "$SCRATCH/batch_ab_progress.log"
            kill_all
            echo "$i,啟動失敗,$layout_n,$fix,$layout_file,啟動失敗" >> "$MANIFEST"
            continue
        fi

        waited=0
        while [ $waited -lt $RUN_TIMEOUT_SEC ]; do
            if grep -q "結束(" "$SCRATCH/grid_ab_${i}.log" 2>/dev/null; then
                break
            fi
            if ! pgrep -f "lib/tennis_bot/grid_patrol_node" > /dev/null 2>&1; then
                break
            fi
            sleep 5
            waited=$((waited + 5))
        done

        reason=$(grep -o "結束([^)]*)" "$SCRATCH/grid_ab_${i}.log" 2>/dev/null | tail -1)
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

        echo "$i,$run_dir,$layout_n,$fix,$layout_file,$reason" >> "$MANIFEST"
        echo "batch $i 結束: run_dir=$run_dir layout=$layout_n fix=$fix reason=$reason $(date '+%H:%M:%S')" >> "$SCRATCH/batch_ab_progress.log"

        kill_all
    done
done

echo "ALL_AB_BATCH_DONE" >> "$SCRATCH/batch_ab_progress.log"
