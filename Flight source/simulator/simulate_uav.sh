#### Simulator Start Script

# bash _run_uav.sh&

# UAV Simulator Script (ArduCopter 3.4dev)
# Note that for this to work, you must have the ardupilot git repo cloned in the
# same directory as the aerowake repo.
cd ~/aerowake_sitl_ws/ardupilot/ArduCopter/
python ../Tools/autotest/sim_vehicle.py -j4 -I 1 -L CMAC --map --console --speedup=1 --aircraft=UAV --out 127.0.0.1:14552
# kill %1 # end the _run_uav command

# FOLLOW THE ARDUPILOT SITL INSTRUCTIONS TO SET UP

#### Manually Enter These Commands

# mode guided
# param set ARMING_CHECK 0
# arm throttle
# takeoff 10

# If the vehicle takes off, you are good to go. 


