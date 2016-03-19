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
#from reel.reel_main import reel_run
from interface.interface import interface_run


# gcs Connection Path. UDP for local simulation. 
gcs_connect_path = '127.0.0.1:14556'
#gcs_connect_path = '/dev/ttyAMA0' #also set baud=57600
#gcs_connect_path = '/dev/ttyUSB0'



#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! SYSTEM SETUP !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Local System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### Logging Setup. ####

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

# Single Log Format:
# logging.basicConfig(filename='system.log',format='%(relativeCreated)s,%(levelname)s: %#(message)s',level=logging.DEBUG)

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


commands_to_interface = Queue()
data_from_interface = Queue()

try:
    ui = interface_run(commands_to_interface,data_from_interface)
except Exception:
    logging.critical('Problem connecting to UI')
    setup_abort("UI Failure")
ui.start()


# #### Start Reel Controller ####
# commands_to_reel = Queue()
# data_from_reel = Queue()
# try:
#     reel = reel_controller(commands_to_airprobe, data_from_airprobe)
# except Exception:
#     logging.critical('Problem connection to reel. Aborting.')
#     setup_abort("Reel System Failure")
#     #sys.exit(1)
# airporbe.start()


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### Pixhawk Connection ####
logging.info("Waiting for Pixhawk")
while True:
    try:
        #Note: connecting another GCS might mess up stream rates. Start mavproxy with --streamrate=-1 to leave stream params alone.
        gcs = connect(gcs_connect_path, heartbeat_timeout=60, rate=20, wait_ready=True)
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


#Log if no MAVLink packets are heard for more than a second
#also log mode changes, and arm/disarm
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

#@gcs.on_message('')

#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Main System !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


'''
Send a STATUSTEXT message. Since the message doesn't have a target_system field,
it is a broadcast message and ardupilot should rebroadcast it to the GCS.
'''
def send_msg_to_gcs(message):
    msg = gcs.message_factory.statustext_encode(mavutil.mavlink.MAV_SEVERITY_CRITICAL, message)
    gcs.send_mavlink(msg)
    gcs.flush()


def abort_mission(reason):
    logging.critical('%s! Aborting mission.' % reason)
    send_msg_to_gcs(reason)
    gcs.mode = VehicleMode("ALTHOLD")
    #TODO: TEST THIS OUT to make sure it doesnt lock us in whatever mode we specify. 

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = gcs.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    gcs.send_mavlink(msg)

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

def set_waypoint(mode,theta,phi,R,tether_l=-1,tether_t=-1,extra=-1):
    cmds = gcs.commands
    cmds.download()
    cmds.wait_ready()
    cmds.clear()
    cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, mode, 0, 0, 0, theta, phi, R)
    cmd3=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, -1,  0, 0, 0, tether_l, tether_t, extra)
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


logging.info("------------------SYSTEM IS READY!!------------------")
logging.info("-----------------------------------------------------")


GCS_cmd = None
G_AUTO = 0
G_TAKEOFF = 1
G_LAND = 2
mode=None
i=0

MISSION_TH  = [1.5, 1.5, 1.5, 1.5,  1.3, 1.3, 1.3, 1.3]
MISSION_PHI = [  0,  .5,   0, -.5,  -.5,   0,  .5,   0]
MISSION_R   = [ 50,  50,  50,  60,   60,  70,  70, 100] 

while True:
    time.sleep(.1)

    #Get Reel Info:
    # try:
    #     reel_reading = data_from_airprobe.get(False)
    # except Empty:
    #     pass

    # ##########################################################################################
    # # Populate GCS State information

    # gcs_tether_l = reel_reading[0]       # GCS Tether Length (m)
    # gcs_tether_tension = reel_reading[1] # GCS Tether Tension (newtons)

    # # Transmit the GCS State information to the UAV:
    # send_state_to_uav([gcs_tether_l,gcs_tether_tension])


    # ##########################################################################################v
    # # Determine flight mode and waypoints for the vehicle


    commands_to_interface.put(000)

    try:
        GCS_cmd = data_from_interface.get(False)
        print GCS_cmd
    except Empty:
        pass



    print "Mode: ",mode," Theta: %.1f  Phi: %.1f  R: %.1f" %(MISSION_TH[i],MISSION_PHI[i],MISSION_R[i])

    # # Modes:
    # # Take Off = Take the vehicle off with slight pitch up.
    # # Auto = Position Control. Needs goal location. 
    # # Land = Landing vehicle: Maintain some throttle and reel the tether in.
    # # None = Do nothing. Initial state. Motors will be on safe, vehicle disarmed. 

    if GCS_cmd == "AUTO_CMD":
        phi = MISSION_PHI[i]
        theta = MISSION_TH[i]
        R = MISSION_R[i]
        mode = G_AUTO
        set_waypoint(mode,theta,phi,R)
        print "Auto Mode"
        print_mission()

    if GCS_cmd == "ADV_CMD":
        i+=1
        if i==len(MISSION_TH):
            i=0

        phi = MISSION_PHI[i]
        theta = MISSION_TH[i]
        R = MISSION_R[i]
        mode = G_AUTO
        set_waypoint(mode,theta,phi,R)
        GCS_cmd = "AUTO_CMD"
        print "Advance Goal Target"
        print_mission()


    if GCS_cmd == "TAKEOFF_CMD":
        phi = 0
        theta = 1.5
        R = 20
        mode = G_TAKEOFF
        set_waypoint(mode,theta,phi,R)
        print "Takeoff Waypoint Set"
        print_mission()

    if GCS_cmd == "LAND_CMD":
        phi = 0
        theta = 1.5
        R = 20
        mode = G_LAND
        set_waypoint(mode,theta,phi,R)
        print "Land Waypoint Set"
        print_mission()



    GCS_cmd="Waiting"
    





    














  
