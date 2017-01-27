#### Simulator Start Script

bash _run_gcs.sh&

# Rover Simulator (ArduRover 3.0.0)
cd ../../../ardupilot/APMrover2/
bash sim_vehicle.sh -j4 -I 2 --map --console --out 127.0.0.1:14554 --out 127.0.0.1:14556
kill %1 # end the _run_gcs command
# map and console

#### Manually Enter These Commands

# mode guided
# arm throttle

#Right click somewhere on the rover map and say fly to here, and the rover should move. 


