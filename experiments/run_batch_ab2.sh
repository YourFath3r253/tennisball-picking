#!/bin/bash
# 第二次A/B測試：3組不同球佈局(0.027kg)，每組各跑一次「無輪速防呆」跟「有輪速防呆」，共6次。
# 機制1(里程計卡住)已經從程式碼刪除，停止條件只剩：撿滿/走完弓字路徑/真實座標撞牆。
# 每次開Gazebo前都先確認+清乾淨 /dev/shm 的DDS殘留檔案，避免累積導致訂閱斷線。
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
source /home/sean/ros2_ws/install/setup.bash

SCRATCH=/tmp/claude-1000/-home-sean-ros2-ws/cfdbfabc-f4d0-4155-bb10-100d98577b2d/scratchpad
LAYOUT_DIR=$SCRATCH/layouts_ab2
mkdir -p "$LAYOUT_DIR"
MANIFEST=$SCRATCH/batch_ab2_manifest.csv
echo "batch_i,run_dir,layout,fix_enabled,layout_file,stop_reason,shm_before_clean,shm_after_clean" > "$MANIFEST"

RUN_TIMEOUT_SEC=700

kill_all() {
    pkill -TERM -f "gzserver.*tennis_court.world" 2>/dev/null
    pkill -TERM -f "gzclient.*tennis_court.world" 2>/dev/null
    pkill -TERM -f "lib/tennis_bot/vision_node" 2>/dev/null
    pkill -TERM -f "lib/tennis_bot/grid_patrol_node" 2>/dev/null
    sleep 3
    pkill -KILL -f "gzserver.*tennis_court.world" 2>/dev/null
    pkill -KILL -f "gzclient.*tennis_court.world" 2>/dev/null
    sleep 2
}

# 確認沒有殘留的DDS共享記憶體檔案，有的話清乾淨；回傳清除前/後的數量方便記錄查證。
clean_shm_and_report() {
    local before after
    before=$(ls /dev/shm/ 2>/dev/null | grep -ic "fastrtps\|cyclone")
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cyclonedds_* 2>/dev/null
    after=$(ls /dev/shm/ 2>/dev/null | grep -ic "fastrtps\|cyclone")
    echo "$before $after"
}

latest_run_dir() {
    ls -d "/home/sean/ros2_ws/experiments/實驗數據"/run* 2>/dev/null | sed 's#.*/run##' | sort -n | tail -1
}

i=0
for layout_n in 1 2 3; do
    layout_file="$LAYOUT_DIR/layout${layout_n}.json"
    python3 experiments/gen_ball_layout.py --mass 0.027 --seed $((300 + layout_n)) --save "$layout_file" > "$SCRATCH/genball_ab2_${layout_n}.log" 2>&1

    for fix in 0 1; do
        i=$((i + 1))
        colcon build --packages-select tennis_bot > "$SCRATCH/build_ab2_${i}.log" 2>&1
        kill_all

        shm_counts=$(clean_shm_and_report)
        shm_before=$(echo "$shm_counts" | cut -d' ' -f1)
        shm_after=$(echo "$shm_counts" | cut -d' ' -f2)
        echo "batch $i 清理殘留檔案: 清除前=$shm_before 清除後=$shm_after $(date '+%H:%M:%S')" >> "$SCRATCH/batch_ab2_progress.log"

        before_n=$(latest_run_dir)
        before_n=${before_n:-0}

        nohup ros2 launch tennis_bot sim_launch.py > "$SCRATCH/gazebo_ab2_${i}.log" 2>&1 &
        gz_waited=0
        while [ $gz_waited -lt 30 ]; do
            if grep -q "wheel transforms" "$SCRATCH/gazebo_ab2_${i}.log" 2>/dev/null; then
                break
            fi
            sleep 2
            gz_waited=$((gz_waited + 2))
        done
        sleep 2
        nohup ros2 run tennis_bot vision_node > "$SCRATCH/vision_ab2_${i}.log" 2>&1 &
        sleep 3
        TENNISBOT_STALE_WHEEL_FIX=$fix \
            nohup ros2 run tennis_bot grid_patrol_node > "$SCRATCH/grid_ab2_${i}.log" 2>&1 &

        echo "batch $i 開始: layout=$layout_n fix=$fix $(date '+%H:%M:%S')" >> "$SCRATCH/batch_ab2_progress.log"

        node_up=0
        for _ in $(seq 1 10); do
            if pgrep -f "lib/tennis_bot/grid_patrol_node" > /dev/null 2>&1; then
                node_up=1
                break
            fi
            sleep 1
        done
        if [ "$node_up" -ne 1 ]; then
            echo "batch $i: grid_patrol_node 沒有成功啟動" >> "$SCRATCH/batch_ab2_progress.log"
            kill_all
            echo "$i,啟動失敗,$layout_n,$fix,$layout_file,啟動失敗,$shm_before,$shm_after" >> "$MANIFEST"
            continue
        fi

        waited=0
        while [ $waited -lt $RUN_TIMEOUT_SEC ]; do
            if grep -q "結束(" "$SCRATCH/grid_ab2_${i}.log" 2>/dev/null; then
                break
            fi
            if ! pgrep -f "lib/tennis_bot/grid_patrol_node" > /dev/null 2>&1; then
                break
            fi
            sleep 5
            waited=$((waited + 5))
        done

        reason=$(grep -o "結束([^)]*)" "$SCRATCH/grid_ab2_${i}.log" 2>/dev/null | tail -1)
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

        echo "$i,$run_dir,$layout_n,$fix,$layout_file,$reason,$shm_before,$shm_after" >> "$MANIFEST"
        echo "batch $i 結束: run_dir=$run_dir layout=$layout_n fix=$fix reason=$reason $(date '+%H:%M:%S')" >> "$SCRATCH/batch_ab2_progress.log"

        kill_all
    done
done

echo "ALL_AB2_BATCH_DONE" >> "$SCRATCH/batch_ab2_progress.log"
