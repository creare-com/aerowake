#!/bin/bash

# Gracefully stop programs before killing screens
screen -S roscore -p 0 -X stuff "^C"

# Kill all screens
# killall screen
