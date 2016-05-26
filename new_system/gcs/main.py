#!/usr/bin/env python2

# GCS BASED MAIN LOOP

import csv
import datetime
import glob
import logging
import math
import os
import subprocess
import sys
import threading
import time
from multiprocessing import Queue
from Queue import Empty

from dronekit import APIException, VehicleMode, connect, mavutil, Command
from reel.reel import reel_run
from interface.interface import interface_run


 #!# All comments to explain system will be prefaced with "#!#"

 #!# Setting up connection path for the Autopilots. 
 #!# For SITL testing, use the following. The UAV is located on Port 14552 and GCS 14554
gcs_connect_path = '127.0.0.1:14556'
gcs_baud = 115200
 
 #!# For Hardware operation, use the following. These baud rates must match those
 #!# as established on the actual hardware. Wired connection should be 115200 and 
 #!# the telemetry radio should be set up for 57600. Uncomment the following lines:

#gcs_connect_path = '/dev/ttyAMA0' #Choose which ever is applicable
#gcs_connect_path = '/dev/ttyS0' #For the RaspPi3 
#gcs_connect_path = '/dev/ttyUSB0'
#gcs_connect_path = '/dev/ttyACM0'
#gcs_baud = 115200



#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! SYSTEM SETUP !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Local System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### Logging Setup. ####

 #!# This logger will create a 'system.log', which will allow debugging later if the UAV system is not 
 #!# not working properly for some reason or another. Messages will be printed to the console, and to the 
 #!# log file. Messages shoudl be priorities with 'info', 'critical', or 'debug'. 

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler('system.log')
fh.setLevel(logging.DEBUG)
ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.DEBUG)
form_fh = logging.Formatter('%(relativeCreated)s,%(levelname)s: %(message)s')
form_ch = logging.Formatter('%(levelname)s: %(message)s')
fh.setFormatter(form_fh)
ch.setFormatter(form_ch)
logger.addHandler(fh)
logger.addHandler(ch)




 #!# This function is called if there is a problem with the vehicle setup. It will kill the script. 


def setup_abort(abort_reason=None):
    t = 0
    #display items on screen
    while t<10:
        logging.critical("System aborting due to error: %s" %abort_reason) 
        time.sleep(1)
        t+=1
    sys.exit(1)
print "\n\n\n\n\n\n"
logging.info("------------ STARTING AEROWAKE SYSTEM ------------")
logging.info("-------------------- GCS NODE --------------------")
time.sleep(2)
#### External Drive Setup ####


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Multiprocessing System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 #!# This block will start both the interface and the reel controller. 
#!# Both of these systems use the multiprocessing infrastructure. 

commands_to_interface = Queue()
data_from_interface = Queue()

try:
    ui = interface_run(commands_to_interface,data_from_interface)
except Exception:
    logging.critical('Problem connecting to UI')
    setup_abort("UI Failure")
ui.start()


#### Start Reel Controller ####
commands_to_reel = Queue()
data_from_reel = Queue()
try:
    reel = reel_run(commands_to_airprobe, data_from_airprobe)
except Exception:
    logging.critical('Problem connection to reel. Aborting.')
    setup_abort("Reel System Failure")
    #sys.exit(1)
reel.start()


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 #!# This block will connect to the GCS pixhawk. The autopilot will be given 60 seconds to connect. 
 #!# If the connection fails, the script will exit. During connection the Pixhawk should be powered on, 
 #!# initialized, and blinking green or yellow. If the script continually fails, reboot the pixhawk and check the paths. 


#### Pixhawk Connection ####
logging.info("Waiting for Pixhawk")
while True:
    try:
        #Note: connecting another GCS might mess up stream rates. Start mavproxy with --streamrate=-1 to leave stream params alone.
        gcs = connect(gcs_connect_path,baud=gcs_baud ,heartbeat_timeout=60, rate=20, wait_ready=True)
        break
    except OSError:
        logging.critical("Cannot find device, is the Pixhawk plugged in? Retrying...")
        time.sleep(5)
    except APIException:
        logging.critical("GCS Pixhawk connection timed out. Retrying...")
logging.info("Pixhawk connected!")



#### System Time Setup ####

# We need to get the time from the gcs (which gets it via gps), because the raspi does not have a RTC
gcs_start_time = 0
rasp_start_time = 0
@gcs.on_message('SYSTEM_TIME')
def gcs_time_callback(self, attr_name, msg):
    global gcs_start_time
    if(gcs_start_time is 0 and msg.time_unix_usec > 0):
        gcs_start_time = msg.time_unix_usec/1000000
        human_time = datetime.datetime.fromtimestamp(gcs_start_time).strftime('%Y-%m-%d %H:%M:%S')
        logging.info("Got GPS lock at %s" % human_time)
        rasp_start_time = time.clock()



#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk Callback/Logging System !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


 #!# Log if no MAVLink packets are heard for more than a second
 #!# Also log mode changes, and arm/disarm
 #!# Normal Dronekit Callbacks. 

timed_out = False

@gcs.on_attribute('last_heartbeat')   
def last_heartbeat_listener(self, attr_name, value):
    if(attr_name is 'last_heartbeat'):
        global timed_out
        if value > 3 and not timed_out:
            timed_out = True
            logging.critical("Pixhawk connection lost!")
        if value < 3 and timed_out:
            timed_out = False;
            logging.info("Pixhawk connection restored.")

@gcs.on_attribute('armed')
def arm_disarm_callback(self,attr_name, msg):
    logging.info("Vehicle is now %sarmed " % ("" if gcs.armed else "dis"))

@gcs.on_attribute('mode')
def mode_callback(self,attr_name, mode):
    logging.info("gcs mode changed to %s" % mode.name)


#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Main System !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


 #!# This is a functions that is part of DroneKit but is currently not used. 
#def send_msg_to_gcs(message):
#    msg = gcs.message_factory.statustext_encode(mavutil.mavlink.MAV_SEVERITY_CRITICAL, message)
#    gcs.send_mavlink(msg)
#    gcs.flush()



# def send_ned_velocity(velocity_x, velocity_y, velocity_z):
#     """
#     Move vehicle in direction based on specified velocity vectors.
#     """
#     msg = gcs.message_factory.set_position_target_local_ned_encode(
#         0,       # time_boot_ms (not used)
#         0, 0,    # target system, target component
#         mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
#         0b0000111111000111, # type_mask (only speeds enabled)
#         0, 0, 0, # x, y, z positions (not used)
#         velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
#         0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
#         0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
#     gcs.send_mavlink(msg)

def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    missionlist=[]
    cmds = gcs.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

 #!# This is the important function that sends commands to the UAV. 
def set_waypoint(mode,theta,phi,L,extra1=-1,tether_t=-1,extra2=-1):
    cmds = gcs.commands
    cmds.download()
    cmds.wait_ready()
    cmds.clear()
    cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, mode, 0, 0, 0, theta, phi, L)
    cmd3=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, -1,  0, 0, 0, extra1, tether_t, extra2)
    cmds.add(cmd1)
    cmds.add(cmd3)
    cmds.upload()

def print_mission():
    missionlist = download_mission()
    for cmd in missionlist:
        #(cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        print cmd.param1
        print cmd.x
        print cmd.y
        print cmd.z

def clear_mission():
    cmds =gcs.commands
    cmds.download()
    cmds.wait_ready()
    cmds.clear()
    cmds.upload()

#!# At this point, arm the GCS pixhawk and place it into guided mode. 
#1# Might have to set the ARMING CHECK parameter to 0 in the GCS pixhawk. 

if not gcs.armed:
    while not gcs.is_armable:
        print "GCS Not Armable"
        time.sleep(.5)
    gcs.armed = True
    while not gcs.armed:
        print 'Waiting for GCS to arm: ',gcs.armed
        time.sleep(1)
gcs.mode = VehicleMode("GUIDED")


#!# System is ready!

logging.info("------------------SYSTEM IS READY!!------------------")
logging.info("-----------------------------------------------------")


#!# These are the different modes that the UAV can be operating in. 
#!# i=0 is the index for the mission. 
GCS_cmd = None
G_AUTO = 0
G_TAKEOFF = 1
G_LAND = 2
mode=None
i=0


#!# This is how the mission is currently specified. Angles are in Radians. 
#!# Theta: 90 degrees is horizontal ( same altitude as boat )
#!# Phi: 0 degrees is straight behind the boat. This is always relative to the stern of the ship.
#!# This system can be replaced with a mission file or similiar. 
MISSION_TH  = [1.2, 1.2, 1.4, 1.2,  1.3, 1.3, 1.3, 1.3]
MISSION_PHI = [  0,  .5,   0, -.5,  -.5,   0,  .5,   0]
MISSION_L   = [ 50,  50,  50,  60,   60,  70,  70, 100] 




#!# Start the main loop. 

while True:
    time.sleep(.2)
    print "Run"
    #Get Reel Info:
    try:
         # Expected to return {"L": <length in meters, as double>, "T": <tension in newtons, as double>}
         reel_reading = data_from_reel.get(False)
    except Empty:
        pass

    # # Determine flight mode and waypoints for the vehicle

    if gcs.armed:
        commands_to_interface.put(001)

        try:
            GCS_cmd = data_from_interface.get(False)
            print GCS_cmd
        except Empty:
            pass

        # # Modes:
        # # Take Off = Take the vehicle off with slight pitch up.
        # # Auto = Position Control. Needs goal location. 
        # # Land = Landing vehicle: Maintain some throttle and reel the tether in.
        # # None = Do nothing. Initial state. Motors will be on safe, vehicle disarmed. 
        if GCS_cmd == "AUTO_CMD":
            i=0
            phi = MISSION_PHI[i]
            theta = MISSION_TH[i]
            L = MISSION_L[i]
            mode = G_AUTO
            set_waypoint(mode,theta,phi,L)
            print "Auto Mode"
            print "Mode: ",mode," Theta: %.1f  Phi: %.1f  L: %.1f" %(MISSION_TH[i],MISSION_PHI[i],MISSION_L[i])
            print_mission()

        if GCS_cmd == "ADV_CMD":
            i+=1
            if i==len(MISSION_TH):
                i=0

            phi = MISSION_PHI[i]
            theta = MISSION_TH[i]
            L = MISSION_L[i]
            mode = G_AUTO
            set_waypoint(mode,theta,phi,L)
            GCS_cmd = "AUTO_CMD"
            print "Advance Goal Target"
            print "Mode: ",mode," Theta: %.1f  Phi: %.1f  L: %.1f" %(MISSION_TH[i],MISSION_PHI[i],MISSION_L[i])
            print_mission()


        if GCS_cmd == "TAKEOFF_CMD":
            phi = 0
            theta = 1.5
            L = 20
            mode = G_TAKEOFF
            set_waypoint(mode,theta,phi,L)
            print "Takeoff Waypoint Set"
            print "Mode: ",mode
            print_mission()

        if GCS_cmd == "LAND_CMD":
            phi = 0
            theta = 1.5
            L = 20
            mode = G_LAND
            set_waypoint(mode,theta,phi,L)
            print "Land Waypoint Set"
            print "Mode: ",mode
            print_mission()



        GCS_cmd="WAITING"

    else:
        print "GCS Not Armed"




    














  
