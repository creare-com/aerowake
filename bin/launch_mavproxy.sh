#!/bin/sh
# Normal operation
# mavproxy.py --master=/dev/ttyAMA0,115200 --out=/dev/ttyUSB0,57600
# Debugging
# Tips: 
# With the below,  you can hit ctrl-C and then get into an interactive python console.
# This gives you access to mpstate -- the main mavproxy object
# For DroneAPI: To get the api and the gcs object into python using:
# >>> dapi = mpstate.modules[-1][0]
# >>> api, gcs = dapi.get_connection()  # This is the same as doing "local_connect()" from the script
cd ../Controller
ipython -i /usr/local/bin/mavproxy.py -- --master=/dev/ttyAMA0,115200 --out=/dev/ttyUSB0,57600 --load-module dronekit_mod --cmd "api start ../Tests/test_drone_api_vehicles.py" --dialect gcs_pixhawk

cd ../bin
