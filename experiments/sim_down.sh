#!/bin/bash
pkill -TERM -f "[g]zserver"
pkill -TERM -f "[g]zclient"
pkill -TERM -f "[l]ib/tennis_bot/"
sleep 3
pkill -KILL -f "[g]zserver"
pkill -KILL -f "[g]zclient"
sleep 1
