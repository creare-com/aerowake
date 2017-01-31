#### Simulator Start Script

bash _run_uav.sh&

# UAV Simulator Script (ArduCopter 3.4dev)
cd ../../../ardupilot/ArduCopter/
bash sim_vehicle.sh -j4 -I 1 --map --console --out 127.0.0.1:14552
kill %1 # end the _run_uav command

# FOLLOW THE ARDUPILOT SITL INSTRUCTIONS TO SET UP

#### Manually Enter These Commands

# mode guided
# arm throttle
# takeoff 10

# If the vehicle takes off, you are good to go. 

