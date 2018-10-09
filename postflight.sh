#!/bin/bash

# Gracefully stop programs
screen -S roscore -p 0 -X stuff "^C"
screen -S yaw-cmd -p 0 -X stuff "^C"
screen -S flight-cmd -p 0 -X stuff "^C"
screen -S airprobe -p 0 -X stuff "^C"
screen -S probe-check -p 0 -X stuff "^C"

# Unmount crearedrive
sleep 2 # Allow ^C to process
umount /crearedrive

FLAGUNMOUNTED="$(lsblk)"
if ! echo "$FLAGUNMOUNTED" | grep -q "crearedrive"
then
	echo "/crearedrive unmounted successfully"
else
	echo "/crearedrive needs to be unmounted"
fi
