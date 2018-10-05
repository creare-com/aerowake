#!/bin/bash

# Gracefully stop programs before killing screens
screen -S roscore -p 0 -X stuff "^C"
screen -S yaw-cmd -p 0 -X stuff "^C"
screen -S flight-cmd -p 0 -X stuff "^C"
screen -S airprobe -p 0 -X stuff "^C"
screen -S probe-check -p 0 -X stuff "^C"

# Kill all screens after allowing ^C to process
sleep 2
killall screen

