#### Simulator Start Script

cd ../../../ardupilot/APMrover2/
#cd ../../../ardupilot/ArduCopter/
sim_vehicle.sh -j4 -I 2 --map --console --out 127.0.0.1:14554 --out 127.0.0.1:14556

# map and console

#### Manually Enter These Commands
# wp load ../Tools/autotest/bos_mission.txt
# arm throttle

# mode auto



