#!/bin/bash
# Sean 的連續 P 控制 (TENNISBOT_STM32_CHASE=p, Kp 0.6) 錄兩支：501 整場 + 501 第一顆球放大
cd /home/sean/ros2_ws
export TENNISBOT_STM32_CHASE=p
bash experiments/demo/record_demo.sh "新車模擬_P控制_kp8000_20261026_seed501" 501 300
STOP_AFTER_BALLS=1 GUI_TRACK_XYZ="0.5 -1.0 0.7" bash experiments/demo/record_demo.sh "新車模擬_P控制_第一顆球放大_seed501" 501 300
echo P_PAIR_DONE
