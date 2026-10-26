#!/bin/bash
# 第三個目標：新車體 v2 跑實體車程式，種子 501~503 (ITF 57.7 g) + 501 (27 g，跟舊車 run180 同條件比較)
cd /home/sean/ros2_ws
LAY=experiments/實驗數據/layouts
for s in 501 502 503; do bash experiments/run_real_car_v2.sh v2_s${s}_58g $LAY/realcar_seed$s.json 900 2>&1 | tail -1; done
BALL_MASS=0.027 bash experiments/run_real_car_v2.sh v2_s501_27g $LAY/realcar_seed501.json 900 2>&1 | tail -1
echo V2_BATCH_DONE
