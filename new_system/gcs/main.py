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

from dronekit import APIException, VehicleMode, connect, mavutil
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




logging.info("------------------SYSTEM IS READY!!------------------")
logging.info("-----------------------------------------------------")


GCS_cmd = None
G_AUTO = 'AUTO_CMD'
G_TAKEOFF = 'TAKEOFF_CMD'
G_LAND = 'LAND_CMD'

while True:
    time.sleep(.1)



    #print " GCS Location:"
    #print [gcs.location.global_frame.lat, gcs.location.global_frame.lon]
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
    except Empty:
        pass

    # # Modes:
    # # Take Off = Take the vehicle off with slight pitch up.
    # # Auto = Position Control. Needs goal location. 
    # # Land = Landing vehicle: Maintain some throttle and reel the tether in.
    # # None = Do nothing. Initial state. Motors will be on safe, vehicle disarmed. 

    if GCS_cmd == "ADV_CMD":
        GCS_cmd = "AUTO_CMD"
        print "Advance Goal Target"

    print GCS_cmd

    # if GCS_cmd == G_AUTO:
    #     goal_pose = [0,80,10] # phi, theta, R in degrees, meters
    #     send_status_to_UAV([GCS_mode,goal_pose])

    # if GCS_cmd == G_TAKEOFF:
    #     r_set = reel_reading[0] # SEt the new goal to a stable location and let the tether wind out. 
    #     initial_wp = [0,80,r_set]
    #     send_status_to_UAV([GCS_mode, initial_wp])
    #     #TODO: Command reel to let tether line out. 

    # if GCS_cmd == G_LAND:
    #     r_prev = reel_reading[0] #Set the new position to a stable location and keep setting the goal R = current reel length
    #     final_wp = [0,70,r_prev]
    #     send_status_to_UAV([GCS_mode, final_wp])
    #     #TODO: Command reel to pull tether line in. 

    # if GCS_cmd == None:
    #     send_status_to_UAV([GCS_mode,[0,0,0]])





    














  
