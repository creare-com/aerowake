#### Simulator Start Script

# bash _run_gcs.sh&

# Rover Simulator (ArduRover 3.0.0)
# Note that for this to work, you must have the ardupilot git repo cloned in the
# same directory as the aerowake repo.
cd ~/aerowake_sitl_ws/ardupilot/APMrover2/
python ../Tools/autotest/sim_vehicle.py -j4 -I 2 -L BriggsField --map --console --out 127.0.0.1:14554 --out 127.0.0.1:14556
# kill %1 # end the _run_gcs command

#### Manually Enter These Commands

# mode guided
# param set ARMING_CHECK 0
# arm throttle

#Right click somewhere on the rover map and say fly to here, and the rover should move. 


