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
		echo "$ROSBAGLOGNAMES"
		echo "The files above will be removed. Continue? [y/N]: " 
		read confirm
		if [ "$confirm" = "y" ]
		then
			# Delete logs after backing up to /tmp, which is autocleaned periodically
			if echo "$UAVLOGNAMES" | grep -q ".log"
			then
				cp /crearedrive/uav-logs/$FILENAME* /tmp
				rm /crearedrive/uav-logs/$FILENAME*
			fi

			if echo "$AIRPROBELOGNAMES" | grep -q ".log"
			then
				cp /crearedrive/airprobe-logs/$FILENAME* /tmp
				rm /crearedrive/airprobe-logs/$FILENAME*
				fi

			if echo "$ROSBAGLOGNAMES" | grep -q ".bag"
			then
				cp /crearedrive/rosbags/$FILENAME* /tmp 
				rm /crearedrive/rosbags/$FILENAME*
			fi
			echo "Files deleted. Deleted files are temporarily available in /tmp"
		else
			echo "Cancelling operation"
		fi
	fi
fi
