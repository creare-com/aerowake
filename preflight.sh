#!/bin/bash

FILENAME=$1

if [ "$FILENAME" = "" ]
then

	echo "Enter filename"

else

	# Kill all existing screens
	killall screen

	# Start all screen sessions in detached mode
	screen -dmS roscore
	screen -dmS yaw-cmd
	screen -dmS flight-cmd
	screen -dmS airprobe
	screen -dmS probe-check

	# Issue commands in these screens
	screen -S roscore -p 0 -X stuff "roscore^M"
	sleep 3 # roscore needs time to initialize
	screen -S yaw-cmd -p 0 -X stuff "roslaunch creare yaw_commanding.launch^M"
	screen -S flight-cmd -p 0 -X stuff "roslaunch aerowake flight_companion.launch filename:=$FILENAME^M"
	screen -S airprobe -p 0 -X stuff "nice -n -10 python ~/creare_ws/src/aerowake/aerowake_git/Flight\ source/uav/airprobe/airprobe_main.py $FILENAME^M"
	sleep 1 # airprobe needs time to initialize
	screen -S probe-check -p 0 -X stuff "tail -f /crearedrive/airprobe-logs/$FILENAME*^M"

fi