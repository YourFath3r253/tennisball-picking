#!/bin/bash
# 新車 v2 跑實車程式 + 螢幕錄影 (跟 影片/47.webm 同樣的版面：Gazebo 視窗 + 車上鏡頭視窗 + UART 終端機)
# 用法: bash experiments/demo/record_demo.sh <影片名稱 (不含副檔名)> [佈局 seed=501] [模擬秒數=600]
# 設定 (環境變數可改)：海綿 kp 8000 (暫定，待實測)、滾輪 W 82 (1218 rpm)、鏡頭 15°、巡邏 120 rpm、球 57.7 g
cd /home/sean/ros2_ws
NAME=$1; SEED=${2:-501}; SECS=${3:-600}
OUT="/home/sean/ros2_ws/影片/$NAME.mp4"
TAG="demo_$NAME"
LOGD=/tmp/claude-1000/-home-sean-ros2-ws/real_car_logs/$TAG
source /opt/ros/humble/setup.bash
export V2_ROLLER_KP=${V2_ROLLER_KP:-8000} ROLLER_W=${ROLLER_W:-82} V2_CAM_PITCH_DEG=${V2_CAM_PITCH_DEG:-15}
export TENNISBOT_PATROL_RPM=${TENNISBOT_PATROL_RPM:-120}
export GUI_CAM_POSE=${GUI_CAM_POSE:-"-19 0 9.5 0 0.55 0"}
# Gazebo 視窗放右邊 (左邊 480 px 給鏡頭畫面 + 終端機)
mkdir -p ~/.gazebo
printf '[geometry]\nx=480\ny=0\nwidth=1440\nheight=1050\n' > ~/.gazebo/gui.ini
GUI=true RUN_TIMEOUT_SEC=4000 bash experiments/run_real_car_v2.sh "$TAG" \
  "experiments/實驗數據/layouts/realcar_seed$SEED.json" "$SECS" > /tmp/claude-1000/-home-sean-ros2-ws/demo_run.log 2>&1 &
# 等裁判節點開始 (車子的兩顆腦隨後啟動)
for _ in $(seq 1 120); do grep -q "開始巡邏" "$LOGD/referee.log" 2>/dev/null && break; sleep 1; done
source install/setup.bash
# 跟車視角 (GUI_TRACK_XYZ)：要等車子 spawn 之後才下，world 檔裡的 <track_visual> 在 GUI 開的時候車子還不在，會沒效
[ -n "$GUI_TRACK_XYZ" ] && { sleep 1; timeout 10 bash experiments/demo/follow_cam.sh tennis_bot; }
# 等 D 的程式開始送 UART 再開兩個顯示視窗；鏡頭視窗沒收到畫面就整組重開 (2026-10-26 第 2 次錄影兩個視窗都沒收到資料)
timeout 60 bash -c 'until ros2 topic echo /jetson_uart --once > /dev/null 2>&1; do sleep 1; done'
VLOG=/tmp/claude-1000/-home-sean-ros2-ws/demo_cam_view.log
for attempt in 1 2 3; do
  python3 experiments/demo/cam_view.py 0 0 480 360 > $VLOG 2>&1 &
  VIEW=$!
  gnome-terminal --geometry=58x30+0+400 --title="Jetson -> STM32 UART" -- bash -c \
    "source /opt/ros/humble/setup.bash; ros2 topic echo /jetson_uart --field data" &
  for _ in $(seq 1 10); do grep -q FIRST_IMAGE $VLOG && break; sleep 1; done
  grep -q FIRST_IMAGE $VLOG && { echo "顯示視窗 OK (第 $attempt 次)"; break; }
  echo "顯示視窗沒收到畫面 (第 $attempt 次)，重開"
  kill $VIEW; pkill -f "[r]os2 topic echo /jetson_uart"; sleep 2
done
# 整理視窗：Claude / Firefox 縮到工作列 (錄完再叫回 Claude)，Gazebo、鏡頭、終端機拉到最上面
for w in Claude firefox nautilus; do DISPLAY=:1 python3 experiments/demo/winctl.py minimize $w > /dev/null; done
for w in Gazebo "Jetson -> STM32" "車上鏡頭"; do DISPLAY=:1 python3 experiments/demo/winctl.py raise "$w" > /dev/null; sleep 0.5; done
# 錄影：X11 整個螢幕 -> H.264 mp4 (GNOME 的 D-Bus 錄影在呼叫的程式結束時就會自動停，不能用)
DISPLAY=:1 gst-launch-1.0 -q -e ximagesrc use-damage=false ! video/x-raw,framerate=20/1 ! videoconvert ! \
  x264enc speed-preset=ultrafast tune=zerolatency bitrate=6000 ! mp4mux ! filesink location="$OUT" > /dev/null 2>&1 &
REC=$!
echo "錄影開始 $(date +%H:%M:%S) -> $OUT"
# STOP_AFTER_BALLS=N：撿到第 N 顆再錄 2.5 s 就停，然後結束這趟 (放大單顆球用)
for _ in $(seq 1 8000); do
  grep -q "結束(" "$LOGD/referee.log" 2>/dev/null && break
  if [ -n "$STOP_AFTER_BALLS" ] && grep -q "($STOP_AFTER_BALLS/10)" "$LOGD/referee.log" 2>/dev/null; then sleep 2.5; break; fi
  sleep 0.5
done
kill -INT $REC; wait $REC 2>/dev/null
[ -n "$STOP_AFTER_BALLS" ] && pkill -INT -f "[l]ib/tennis_bot/real_car_node"
echo "錄影結束 $(date +%H:%M:%S)：$(grep -o '結束([^)]*)[^$]*' "$LOGD/referee.log" | tail -1)"
kill $VIEW 2>/dev/null
pkill -f "[r]os2 topic echo /jetson_uart"
wait
tail -1 /tmp/claude-1000/-home-sean-ros2-ws/demo_run.log
printf '[geometry]\nx=0\ny=0\n' > ~/.gazebo/gui.ini
DISPLAY=:1 python3 experiments/demo/winctl.py raise Claude > /dev/null
echo RECORD_DONE
