#!/bin/bash
cd ~/raspilot
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1:$LD_PRELOAD
export PYTHONPATH="$PWD"
pkill -f transcoderd
python3.7 selfdrive/controls/transcoderd.py &
pkill -f controlsd
pkill -f pandad
pkill -f boardd
pkill -f ubloxd
pkill -f dashboard
python ~/raspilot/selfdrive/controls/controlsd.py & 
python ~/raspilot/selfdrive/pandad.py &
#~/raspilot/selfdrive/boardd/boardd &
~/raspilot/selfdrive/locationd/ubloxd &
python ~/raspilot/dashboard.py &
