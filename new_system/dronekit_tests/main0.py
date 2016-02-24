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


logging.info("------------ STARTING AEROWAKE SYSTEM ------------")
logging.info("-------------------- UAV NODE 0 ------------------")


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### Pixhawk Connection ####
logging.info("Waiting for Pixhawk")
while True:
    try:
        #Note: connecting another GCS might mess up stream rates. Start mavproxy with --streamrate=-1 to leave stream params alone.
        autopilot = connect(autopilot_connect_path, heartbeat_timeout=60, rate=40, wait_ready=True)
        break
    except OSError:
        logging.critical("Cannot find device, is the Pixhawk plugged in? Retrying...")
        time.sleep(5)
    except APIException:
        logging.critical("Pixhawk connection timed out. Retrying...")
logging.info("Pixhawk connected!")

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
    logging.warning("Autopilot reports arming checks are not standard!")


#### System Time Setup ####

# We need to get the time from the autopilot (which gets it via gps), because the raspi does not have a RTC
autopilot_start_time = 0
rasp_start_time = 0
@autopilot.on_message('SYSTEM_TIME')
def autopilot_time_callback(self, attr_name, msg):
    global autopilot_start_time
    if(autopilot_start_time is 0 and msg.time_unix_usec > 0):
        autopilot_start_time = msg.time_unix_usec/1000000
        human_time = datetime.datetime.fromtimestamp(autopilot_start_time).strftime('%Y-%m-%d %H:%M:%S')
        logging.info("Got GPS lock at %s" % human_time)
        rasp_start_time = time.clock()



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
            logging.critical("Pixhawk connection lost!")
        if value < 3 and timed_out:
            timed_out = False;
            logging.info("Pixhawk connection restored.")

@autopilot.on_attribute('armed')
def arm_disarm_callback(self,attr_name, msg):
    logging.info("Vehicle is now %sarmed " % ("" if autopilot.armed else "dis"))

@autopilot.on_attribute('mode')
def mode_callback(self,attr_name, mode):
    logging.info("Autopilot mode changed to %s" % mode.name)

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


logging.info("------------------SYSTEM IS READY!!------------------")
logging.info("-----------------------------------------------------")
t0=datetime.datetime.now()
last_location = autopilot.location.global_frame.lat
while True:
    time.sleep(.01)

    current_location = autopilot.location.global_frame.lat
    if current_location!=last_location:
        last_location = current_location
        t1 = datetime.datetime.now()
        print (t1-t0).total_seconds()
        t0=datetime.datetime.now()



    #print "Global Location: %s" %autopilot.location.global_frame.lat







  
