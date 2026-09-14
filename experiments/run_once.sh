#!/bin/bash
# 跑一次完整模擬 (目前 tennis_court.world 的球佈局)：清 /dev/shm 殘留 → 開 Gazebo →
# vision_node → grid_patrol_node → 等 結束(...) → 畫圖。
# 用法: bash experiments/run_once.sh [tag]   log 會放在 scratchpad，檔名帶 tag。
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
source /home/sean/ros2_ws/install/setup.bash

TAG=${1:-once}
SCRATCH=/tmp/claude-1000/-home-sean-ros2-ws/cfdbfabc-f4d0-4155-bb10-100d98577b2d/scratchpad
mkdir -p "$SCRATCH"
RUN_TIMEOUT_SEC=700

kill_all() {
    pkill -TERM -f "[g]zserver"
    pkill -TERM -f "[g]zclient"
    pkill -TERM -f "[l]ib/tennis_bot/vision_node"
    pkill -TERM -f "[l]ib/tennis_bot/grid_patrol_node"
    sleep 3
    pkill -KILL -f "[g]zserver"
    pkill -KILL -f "[g]zclient"
    sleep 2
}

latest_run_dir() {
    ls -d "/home/sean/ros2_ws/experiments/實驗數據"/run* 2>/dev/null | sed 's#.*/run##' | sort -n | tail -1
}

kill_all
shm_before=$(ls /dev/shm/ 2>/dev/null | grep -ic "fastrtps\|cyclone")
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cyclonedds_* 2>/dev/null
shm_after=$(ls /dev/shm/ 2>/dev/null | grep -ic "fastrtps\|cyclone")
echo "[$TAG] shm 殘留 清除前=$shm_before 清除後=$shm_after"

before_n=$(latest_run_dir); before_n=${before_n:-0}

nohup ros2 launch tennis_bot sim_launch.py > "$SCRATCH/gazebo_${TAG}.log" 2>&1 &
for _ in $(seq 1 15); do
    grep -q "wheel transforms" "$SCRATCH/gazebo_${TAG}.log" 2>/dev/null && break
    sleep 2
done
sleep 2
# 實驗用：GZ_UPDATE_RATE=700 會把物理更新率從預設 1000 壓到 700，即時率變 ~0.7，
# 用來驗證里程計 dt 來源 (封包時間戳 vs 真實時鐘) 對即時率的敏感度。
if [ -n "$GZ_UPDATE_RATE" ]; then
    gz physics -u "$GZ_UPDATE_RATE" && echo "[$TAG] 物理更新率設為 $GZ_UPDATE_RATE"
fi
nohup ros2 run tennis_bot vision_node > "$SCRATCH/vision_${TAG}.log" 2>&1 &
sleep 3
nohup ros2 run tennis_bot grid_patrol_node > "$SCRATCH/grid_${TAG}.log" 2>&1 &

node_up=0
for _ in $(seq 1 10); do
    pgrep -f "[l]ib/tennis_bot/grid_patrol_node" > /dev/null && { node_up=1; break; }
    sleep 1
done
if [ "$node_up" -ne 1 ]; then
    echo "[$TAG] grid_patrol_node 沒有啟動"; kill_all; exit 1
fi
echo "[$TAG] 開始 $(date '+%H:%M:%S')"

waited=0
while [ $waited -lt $RUN_TIMEOUT_SEC ]; do
    grep -q "結束(" "$SCRATCH/grid_${TAG}.log" 2>/dev/null && break
    pgrep -f "[l]ib/tennis_bot/grid_patrol_node" > /dev/null || break
    sleep 5
    waited=$((waited + 5))
done
reason=$(grep -o "結束([^)]*)[^$]*" "$SCRATCH/grid_${TAG}.log" 2>/dev/null | tail -1)
pkill -INT -f "[l]ib/tennis_bot/grid_patrol_node"
sleep 3
after_n=$(latest_run_dir); after_n=${after_n:-0}
run_dir="run${after_n}"
[ "$after_n" -gt "$before_n" ] || run_dir="unknown"
echo "[$TAG] 結束 run_dir=$run_dir reason=${reason:-逾時或異常} $(date '+%H:%M:%S')"
kill_all
if [ "$run_dir" != "unknown" ]; then
    if [ -n "$LAYOUT_JSON" ]; then
        # 隨機佈局：畫圖時用該佈局的球位置，不用 plot_run 裡寫死的標準佈局
        python3 -c "
import json, sys; sys.path.insert(0, 'experiments')
from plot_run import plot_run
plot_run('experiments/實驗數據/$run_dir', balls={k: tuple(v) for k, v in json.load(open('$LAYOUT_JSON')).items()})"
    else
        python3 experiments/plot_run.py "experiments/實驗數據/$run_dir"
    fi
fi
echo "RUN_ONCE_DONE $run_dir"
