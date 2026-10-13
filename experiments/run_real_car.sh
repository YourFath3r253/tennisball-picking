#!/bin/bash
# 實體車模式跑一次 (2026-10-12)：Gazebo + jetson_d_node (D 同學程式 SIL) + stm32_b_node (B 同學 main.c 移植)
# + real_car_node (裁判：起點、滾輪、撿球判定、軌跡)。結束後把 Jetson / STM32 的 CSV 搬進 runN 資料夾。
# 用法: bash experiments/run_real_car.sh <tag>
# 環境變數: WORLD (預設 indoor_court.world；教室對照用 classroom_floor.world)、LAYOUT_JSON (隨機佈局)、GUI (預設 false)、
#           TENNISBOT_STM32_CHASE (b|p)、TENNISBOT_BOUNCE_TIMEOUT (模擬秒數，預設 900)、RUN_TIMEOUT_SEC (真實秒數)
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
source /home/sean/ros2_ws/install/setup.bash
TAG=${1:-realcar}
SCRATCH=/tmp/claude-1000/-home-sean-ros2-ws/real_car_logs/$TAG
rm -rf "$SCRATCH"; mkdir -p "$SCRATCH"
WORLD=${WORLD:-indoor_court.world}
GUI=${GUI:-false}
RUN_TIMEOUT_SEC=${RUN_TIMEOUT_SEC:-1500}

kill_all() {
    pkill -TERM -f "[g]zserver"; pkill -TERM -f "[g]zclient"; pkill -TERM -f "[l]ib/tennis_bot/"
    sleep 3
    pkill -KILL -f "[g]zserver"; pkill -KILL -f "[g]zclient"; pkill -KILL -f "[l]ib/tennis_bot/"
    pkill -TERM -f "[r]os2 launch tennis_bot"; pkill -TERM -f "[r]obot_state_publisher"; sleep 1
    pkill -KILL -f "[r]os2 launch tennis_bot"; pkill -KILL -f "[r]obot_state_publisher"
    sleep 2
}
latest_run_dir() { ls -d /home/sean/ros2_ws/experiments/實驗數據/run* 2>/dev/null | sed 's#.*/run##' | sort -n | tail -1; }

kill_all
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cyclonedds_* 2>/dev/null
before_n=$(latest_run_dir); before_n=${before_n:-0}
# Gazebo 偶爾在 spawn 時 segfault (exit -11)：沒看到 "wheel transforms" 就整個重開，最多 3 次
for attempt in 1 2 3; do
    nohup ros2 launch tennis_bot sim_launch.py world:="$WORLD" gui:="$GUI" > "$SCRATCH/gazebo.log" 2>&1 &
    up=0
    for _ in $(seq 1 25); do grep -q "wheel transforms" "$SCRATCH/gazebo.log" 2>/dev/null && { up=1; break; }; sleep 2; done
    [ $up -eq 1 ] && ! grep -q "process has died" "$SCRATCH/gazebo.log" && break
    echo "[$TAG] Gazebo 沒起來 (第 $attempt 次)，重開"; kill_all
done
sleep 2
if [ -n "$LAYOUT_JSON" ]; then
    export TENNISBOT_BALL_NAMES=$(python3 -c "import json,sys; print(','.join(json.load(open(sys.argv[1])).keys()))" "$LAYOUT_JSON")
fi
# 裁判先開 (傳送到起點)，車子的兩顆「腦」再開
nohup ros2 run tennis_bot real_car_node > "$SCRATCH/referee.log" 2>&1 &
sleep 4
TENNISBOT_STM32_LOG_DIR="$SCRATCH" nohup ros2 run tennis_bot stm32_b_node > "$SCRATCH/stm32.log" 2>&1 &
TEST_NAME=sim TENNISBOT_JETSON_LOG_DIR="$SCRATCH" nohup ros2 run tennis_bot jetson_d_node > "$SCRATCH/jetson.log" 2>&1 &
echo "[$TAG] 開始 $(date '+%H:%M:%S') world=$WORLD"
waited=0
while [ $waited -lt $RUN_TIMEOUT_SEC ]; do
    grep -q "結束(" "$SCRATCH/referee.log" 2>/dev/null && break
    pgrep -f "[l]ib/tennis_bot/real_car_node" > /dev/null || break
    sleep 5; waited=$((waited + 5))
done
reason=$(grep -o "結束([^)]*)[^$]*" "$SCRATCH/referee.log" 2>/dev/null | tail -1)
pkill -INT -f "[l]ib/tennis_bot/real_car_node"; pkill -INT -f "[l]ib/tennis_bot/stm32_b_node"; pkill -INT -f "[l]ib/tennis_bot/jetson_d_node"
sleep 3
after_n=$(latest_run_dir); after_n=${after_n:-0}
run_dir="run${after_n}"; [ "$after_n" -gt "$before_n" ] || run_dir="unknown"
kill_all
if [ "$run_dir" != "unknown" ]; then
    D="/home/sean/ros2_ws/experiments/實驗數據/$run_dir"
    cp "$SCRATCH"/sim.csv "$SCRATCH"/sim_court.csv "$SCRATCH"/stm32.csv "$SCRATCH"/stm32_events.csv "$D"/ 2>/dev/null
    cp "$SCRATCH"/jetson.log "$SCRATCH"/stm32.log "$SCRATCH"/referee.log "$D"/ 2>/dev/null
    { echo "world=$WORLD"; echo "chase=${TENNISBOT_STM32_CHASE:-b}"; echo "layout=$LAYOUT_JSON"; } >> "$D/meta.txt"
fi
stalls=$(grep -c "相機畫面中斷" "$SCRATCH/jetson.log" 2>/dev/null)
echo "[$TAG] 結束 run_dir=$run_dir reason=${reason:-逾時或異常} 相機中斷=${stalls:-0} $(date '+%H:%M:%S')"
echo "RUN_REAL_CAR_DONE $run_dir"
