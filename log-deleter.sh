#!/bin/bash

FILENAME=$1
DRIVETEST="$(lsblk)"

if [ "$FILENAME" = "" ]
then
	echo "Enter filename to delete"
else
	# Check for crearedrive
	if ! echo "$DRIVETEST" | grep -q "crearedrive"
	then
		echo "Insert crearedrive"
	else
		# Gather logs with given filenames
		UAVLOGNAMES="$(ls /crearedrive/uav-logs/$FILENAME*)"
		AIRPROBELOGNAMES="$(ls /crearedrive/airprobe-logs/$FILENAME*)"
		ROSBAGLOGNAMES="$(ls /crearedrive/rosbags/$FILENAME*)"
		echo "$UAVLOGNAMES"
		echo "$AIRPROBELOGNAMES"
		echo "The files above will be removed. Continue? [y/N]: " 
		read confirm
		if [ "$confirm" = "y" ]
		then
			rm /crearedrive/uav-logs/$FILENAME*
			rm /crearedrive/airprobe-logs/$FILENAME*
			rm /crearedrive/rosbags/$FILENAME*
			echo "Files deleted."
		else
			echo "Cancelling operation"
		fi
	fi
fi
