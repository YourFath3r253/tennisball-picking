#!/bin/bash
# 只開 Gazebo (+ 可選 vision_node)，給單項物理/控制小實驗用 (ball_roll_test / pickup_test / step_response_test)。
# 用法: bash experiments/sim_up.sh [with_vision]     關掉: bash experiments/sim_down.sh
# 可選環境變數: WORLD、GUI (預設 false)
cd /home/sean/ros2_ws
source /opt/ros/humble/setup.bash
source /home/sean/ros2_ws/install/setup.bash
LOG=/tmp/claude-1000/-home-sean-ros2-ws/sim_up_logs
mkdir -p "$LOG"
bash /home/sean/ros2_ws/experiments/sim_down.sh
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cyclonedds_* 2>/dev/null
nohup ros2 launch tennis_bot sim_launch.py world:="${WORLD:-tennis_court.world}" gui:="${GUI:-false}" > "$LOG/gazebo.log" 2>&1 &
for _ in $(seq 1 20); do
    grep -q "wheel transforms" "$LOG/gazebo.log" 2>/dev/null && break
    sleep 2
done
sleep 2
if [ "$1" = "with_vision" ]; then
    nohup ros2 run tennis_bot vision_node > "$LOG/vision.log" 2>&1 &
    sleep 3
fi
echo "SIM_UP (log: $LOG)"
