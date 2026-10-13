#!/bin/bash
pkill -TERM -f "[g]zserver"
pkill -TERM -f "[g]zclient"
pkill -TERM -f "[l]ib/tennis_bot/"
sleep 3
pkill -KILL -f "[g]zserver"
pkill -KILL -f "[g]zclient"
sleep 1
# 2026-10-12：ros2 launch 本身跟 robot_state_publisher 也要關，不然會殘留 (舊 URDF 的 TF 一直發)
pkill -TERM -f "[r]os2 launch tennis_bot"
pkill -TERM -f "[r]obot_state_publisher"
sleep 1
pkill -KILL -f "[r]os2 launch tennis_bot"
pkill -KILL -f "[r]obot_state_publisher"
