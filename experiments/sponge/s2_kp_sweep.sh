#!/bin/bash
# S2 海綿剛性掃描 (舊車)：只改滾輪 <kp> (simple_bot.urdf 左右滾輪)，其他照舊；結束後 kp 改回 150
# 用法: bash experiments/sponge/s2_kp_sweep.sh <out_dir> "<kp們>" [W=82] [offsets=0,0.03,-0.03] [v=0.41]
cd /home/sean/ros2_ws
OUT=$1; KPS=$2; W=${3:-82}; OFFS=${4:-0,0.03,-0.03}; V=${5:-0.41}
U=src/tennis_bot/urdf/simple_bot.urdf
for kp in $KPS; do
  sed -i "s#<kp>[0-9.]*</kp>#<kp>$kp</kp>#g" $U
  echo "kp=$kp ($(grep -c "<kp>$kp</kp>" $U) 個滾輪)"
  bash experiments/sponge/s2_old_car.sh "$OUT/kp$kp" "$W" "$OFFS" "$V" 2>&1 | grep "^trial" | sed "s/^/  kp=$kp /"
done
sed -i "s#<kp>[0-9.]*</kp>#<kp>150</kp>#g" $U
echo KP_SWEEP_DONE
