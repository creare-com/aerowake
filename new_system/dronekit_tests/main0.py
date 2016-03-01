#!/usr/bin/env python2

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


# Autopilot Connection Path. UDP for local simulation. 
autopilot_connect_path = 'udpin:0.0.0.0:14550'
#autopilot_connect_path = '/dev/ttyAMA0' #also set baud=57600
#autopilot_connect_path = '/dev/ttyUSB0'



#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! SYSTEM SETUP !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Local System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### Logging Setup. ####

# Single Log Format:
# logging.basicConfig(filename='system.log',format='%(relativeCreated)s,%(levelname)s: %#(message)s',level=logging.DEBUG)

print("------------ STARTING AEROWAKE SYSTEM ------------")
print("-------------------- UAV NODE 0 ------------------")


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### Pixhawk Connection ####
print("Waiting for Pixhawk")
while True:
    try:
        #Note: connecting another GCS might mess up stream rates. Start mavproxy with --streamrate=-1 to leave stream params alone.
        autopilot = connect(autopilot_connect_path, heartbeat_timeout=60, rate=20, wait_ready=True)
        break
    except OSError:
        print("Cannot find device, is the Pixhawk plugged in? Retrying...")
        time.sleep(5)
    except APIException:
        print("Pixhawk connection timed out. Retrying...")
print("Pixhawk connected!")

if(autopilot.parameters['ARMING_CHECK'] != 1):
    print("Autopilot reports arming checks are not standard!")


#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk Callback/Logging System !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


#Log if no MAVLink packets are heard for more than a second
#also log mode changes, and arm/disarm
timed_out = False

@autopilot.on_attribute('last_heartbeat')   
def last_heartbeat_listener(self, attr_name, value):
    if(attr_name is 'last_heartbeat'):
        global timed_out
        if value > 3 and not timed_out:
            timed_out = True
            print("Pixhawk connection lost!")
        if value < 3 and timed_out:
            timed_out = False;
            print("Pixhawk connection restored.")

@autopilot.on_attribute('armed')
def arm_disarm_callback(self,attr_name, msg):
    print("Vehicle is now %sarmed " % ("" if autopilot.armed else "dis"))

@autopilot.on_attribute('mode')
def mode_callback(self,attr_name, mode):
    print("Autopilot mode changed to %s" % mode.name)

#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Main System !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


'''
Send a STATUSTEXT message. Since the message doesn't have a target_system field,
it is a broadcast message and ardupilot should rebroadcast it to the GCS.
'''
def send_msg_to_gcs(message):
    msg = autopilot.message_factory.statustext_encode(mavutil.mavlink.MAV_SEVERITY_CRITICAL, message)
    autopilot.send_mavlink(msg)
    autopilot.flush()

def goto_position_target_local_ned(north, east, down):
    """ 
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = autopilot.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    autopilot.send_mavlink(msg)


print("------------------SYSTEM IS READY!!------------------")
print("-----------------------------------------------------")

print "Arming motors"
# Copter should arm in GUIDED mode
autopilot.mode    = VehicleMode("GUIDED")
autopilot.armed   = True    

while not autopilot.armed:      
    print " Waiting for arming..."
    time.sleep(1)

while True:
    time.sleep(.1)

    goto_position_target_local_ned(10,10,-100)


    











  
