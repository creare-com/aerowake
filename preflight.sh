#!/bin/bash

FILENAME=$1
DRIVETEST="$(lsblk)"

if [ "$FILENAME" = "" ]
then
	echo "Enter filename"
else
	# Check for crearedrive
	if ! echo "$DRIVETEST" | grep -q "crearedrive"
	then
		echo "Insert crearedrive"
	else
		# Ensure filename has not been used yet
		UAVFILES="$(ls /crearedrive/uav-logs/)"
		AIRPROBEFILES="$(ls /crearedrive/airprobe-logs/)"
		ROSBAGFILES="$(ls /crearedrive/rosbags/)"
		FILENAMETEST="$UAVFILES$AIRPROBEFILES$ROSBAGFILES"
		echo "$FILENAMETEST"
		if echo "$FILENAMETEST" | grep -q "$FILENAME"
		then
		  echo "Pick a different filename (already used)"
		else

			# Kill all existing screens
			killall screen

			# Start all screen sessions in detached mode
			screen -dmS roscore
			screen -dmS yaw-cmd
			screen -dmS flight-cmd
			screen -dmS airprobe
			screen -dmS probe-check

			# Issue commands in screen sessions
			screen -S roscore -p 0 -X stuff "roscore^M"
			sleep 3 # roscore needs time to initialize
			screen -S yaw-cmd -p 0 -X stuff "roslaunch creare yaw_commanding.launch^M"
			screen -S flight-cmd -p 0 -X stuff "roslaunch aerowake flight_companion.launch filename:=$FILENAME^M"
			screen -S airprobe -p 0 -X stuff "nice -n -10 python ~/creare_ws/src/aerowake/aerowake_git/Flight\ source/uav/airprobe/airprobe_main.py $FILENAME^M"
			sleep 1 # airprobe needs time to initialize
			screen -S probe-check -p 0 -X stuff "tail -f /crearedrive/airprobe-logs/$FILENAME*^M"

			# Reattach to flight-cmd as that is most-likely desired screen
			screen -r flight-cmd

		fi
	fi
fi
