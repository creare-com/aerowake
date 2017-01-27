#!/bin/bash

# This script shouldn't be called directly.  It's only a different file so we
# can easily run it in the background.

# Wait for the other guy to finish booting.  We can't perform a foreground wait
# in the other script because the other script opens to an interactive console.
sleep 30
cd ../gcs/
python main.py >> uav.log
