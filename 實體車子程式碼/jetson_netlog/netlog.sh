#!/bin/bash
# Jetson WiFi 連線記錄器：每筆同時 ping 熱點(手機)跟外網，
# gw_ok=0 代表 Jetson↔手機 的 WiFi 斷了；gw_ok=1 但 inet_ok=0 代表手機本身的對外網路斷了。
# uptime_s 是開機後秒數：Jetson 沒有 RTC，開機到 NTP 校時前的 time 欄位不準，要以 uptime_s 對時間線。

INTERVAL=5
LOGDIR=/home/hp/netlog
INET_HOST=1.1.1.1

mkdir -p "$LOGDIR"

ping_ms() {
    ping -c1 -W1 "$1" 2>/dev/null | awk -F'time=' '/time=/ {split($2, a, " "); print a[1]}'
}

while true; do
    f="$LOGDIR/netlog_$(date +%Y%m%d).csv"
    [ -f "$f" ] || echo "time,uptime_s,ssid,bssid,signal_dbm,power_save,gw,gw_ok,gw_ms,inet_ok,inet_ms" > "$f"

    up=$(cut -d' ' -f1 /proc/uptime)
    link=$(iw dev wlan0 link 2>/dev/null)
    ssid=$(echo "$link" | awk -F': ' '/SSID/ {print $2}' | tr ',' '_')
    bssid=$(echo "$link" | awk '/Connected to/ {print $3}')
    sig=$(echo "$link" | awk '/signal/ {print $2}')
    ps=$(iw dev wlan0 get power_save 2>/dev/null | awk '{print $3}')
    gw=$(ip route | awk '/^default/ && /wlan0/ {print $3; exit}')

    gw_ms=""
    [ -n "$gw" ] && gw_ms=$(ping_ms "$gw")
    inet_ms=$(ping_ms "$INET_HOST")
    gw_ok=0; [ -n "$gw_ms" ] && gw_ok=1
    inet_ok=0; [ -n "$inet_ms" ] && inet_ok=1

    echo "$(date '+%Y-%m-%d %H:%M:%S'),$up,$ssid,$bssid,$sig,$ps,$gw,$gw_ok,$gw_ms,$inet_ok,$inet_ms" >> "$f"
    sleep "$INTERVAL"
done
