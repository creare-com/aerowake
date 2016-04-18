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

gcs_connect_path = 'udpin:0.0.0.0:14551'



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
        autopilot = connect(autopilot_connect_path, heartbeat_timeout=60, rate=40, wait_ready=True)
        break
    except OSError:
        print("Cannot find device, is the Pixhawk plugged in? Retrying...")
        time.sleep(5)
    except APIException:
        print("Pixhawk connection timed out. Retrying...")
print("Pixhawk connected!")

"""logging.info("Waiting for GCS")
while True:
    try:
        gcs = connect(gcs_connect_path,heartbeat_timeout=60,rate=20,wait_ready=True)
        break
    except OSError:
        logging.critical("Cannot find device, is the GCS plugged in? Retrying...")
        time.sleep(5)
    except APIException:
        logging.critical("GCS connection timed out. Retrying...")
logging.info("GCS connected!")"""







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

def condition_yaw(heading):
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    autopilot.send_mavlink(msg)
    autopilot.flush()


def send_attitude_msg(roll,pitch,yaw,thr):

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    #type(pose) = geometry_msgs.msg.Pose
    quat = [1,0,0,0]

    msg = autopilot.message_factory.set_attitude_target(
        0,               #Field Name  Type    Description
        0,                #time_boot_ms    uint32_t    Timestamp in milliseconds since system boot
        0,                #target_system   uint8_t System ID
        0,                #target_component    uint8_t Component ID
        0,                #type_mask   uint8_t Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
        quat,                #q   float[4]    Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        0,                #body_roll_rate  float   Body roll rate in radians per second
        0,                #body_pitch_rate float   Body roll rate in radians per second
        0,                #body_yaw_rate   float   Body roll rate in radians per second
        thr)                #thrust  float   Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)


    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.
t0=datetime.datetime.now()
last_location = autopilot.location.global_frame.lat
    At time of writing, acceleration and yaw bits are ignored.
    time.sleep(.01)
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
while True:
    time.sleep(.1)

    goto_position_target_local_ned(10,10,-100)



    current_location = autopilot.location.global_frame.lat
    if current_location!=last_location:
        last_location = current_location
        t1 = datetime.datetime.now()
        print (t1-t0).total_seconds()
        t0=datetime.datetime.now()



    #print "Global Location: %s" %autopilot.location.global_frame.lat







  
