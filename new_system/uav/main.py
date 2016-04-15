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
import numpy as np

from dronekit import APIException, VehicleMode, connect, mavutil

from airprobe.airprobe_main import airprobe_run

from controller.pose_control import pose_controller_class

# Autopilot Connection Path. UDP for local simulation. 
#autopilot_connect_path = 'udpin:127.0.0.1:14550'
#gcs_connect_path = 'udpin:127.0.0.1:14554'
#autopilot_connect_path = '/dev/ttyAMA0' #also set baud=57600
#autopilot_connect_path = '/dev/ttyUSB0'


autopilot_connect_path = '127.0.0.1:14552'
#autopilot_connect_path = '/dev/ttyAMA0'
#uav_baud = 115200

gcs_connect_path = '127.0.0.1:14554'
#gcs_connect_path = '/dev/ttyUSB0'
#gcs_baud = 57600


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
print "\n\n\n\n"
logging.info("------------ STARTING AEROWAKE SYSTEM ------------")
logging.info("-------------------- UAV NODE --------------------")


#### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Start Position Controller !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

CONTROL_DT = .1

pose_controller = pose_controller_class(CONTROL_DT)


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Multiprocessing System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


#### Start Air Probe Controller ####
# commands_to_airprobe = Queue()
# data_from_airprobe = Queue()
# try:
#     airprobe = airprobe_main(commands_to_airprobe, data_from_airprobe)
# except Exception:
#     logging.critical('Problem connection to airprobe. Aborting.')
#     setup_abort("Airprobe System Failure")
#     #sys.exit(1)
# airprobe.start()


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### Autopilot Connection ####
logging.info("Waiting for Autopilot")
while True:
    try:
        #Note: connecting another GCS might mess up stream rates. Start mavproxy with --streamrate=-1 to leave stream params alone.
        autopilot = connect(autopilot_connect_path, heartbeat_timeout=60, rate=20, wait_ready=True)
        break
    except OSError:
        logging.critical("Cannot find device, is the Autopilot plugged in? Retrying...")
        time.sleep(5)
    except APIException:
        logging.critical("Autopilot connection timed out. Retrying...")
logging.info("Autopilot connected!")

if(autopilot.parameters['ARMING_CHECK'] != 1):
    logging.warning("Autopilot reports arming checks are not standard!")

#### GCS Connection ####
logging.info("Waiting for GCS")
while True:
    try:
        gcs = connect(gcs_connect_path,heartbeat_timeout=60, rate=20, wait_ready=True)
        break
    except OSError:
        logging.critical("Cannot find device, is the GCS connected? Retrying...")
        time.sleep(5)
    except APIException:
        logging.critical("GCS connection timed out. Retrying...")
logging.info("GCS Connected")



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

# @autopilot.on_message('NAV_CONTROLLER_OUTPUT')
# def message_callback(self,attr_name,mode):
#     print mode
#     print mode

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


def abort_mission(reason):
    logging.critical('%s! Aborting mission.' % reason)
    send_msg_to_gcs(reason)
    autopilot.mode = VehicleMode("ALTHOLD")
    #TODO: TEST THIS OUT to make sure it doesnt lock us in whatever mode we specify. 


# This function is used to set the attitude of the vehicle. The bitmask is set here to take in a quaternion and throttle setting from 0-1. This throttle setting is that same as altitude hold mode. 
def set_attitude_target(data_in):
    quat = data_in[0:4]
    thr = data_in[4]
    msg = autopilot.message_factory.set_attitude_target_encode(
        0, 0,0,
        0b000000001,     # bitmask
        quat,    # quat
        0,              #roll rate
        0,              #  pitch rate
        0,              # yaw speed deg/s
        thr)             # thrust
    autopilot.send_mavlink(msg)


def condition_yaw(heading, relative=False):
    if heading<0:
        heading+=360

    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = autopilot.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    autopilot.send_mavlink(msg)

# This function downloads all of the current waypoints from the GCS pixhawk (if there are any), clears the GCS pixhawk, and returns to mission information. 
def download_mission():
    """ Downloads the current mission and returns it in a list. """
    missionlist=[]
    cmds = gcs.commands
    cmds.download()
    #cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
        #print "I See A Waypoint!"
    for cmd in cmds:
        cmds.clear()
        cmds.upload()
        #print "I Cleared The Waypoint"
    return missionlist

# This function takes the mission list from the  GCS pixhawk, and parses it into a command. This system uses 2 waypoints to specify a command from the ground station. 
# First waypoint holds goal and mode information.
# Second waypoint holds tether information, and there is room for two extra parameters. 
def read_mission():

    missionlist = download_mission()

    data=[]
    for cmd in missionlist:
        #(cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        data.append(cmd.param1)
        data.append(cmd.x)
        data.append(cmd.y)
        data.append(cmd.z)

    if data!=[]:
        print  data
        pose_controller.set_goal(data[1],data[2],data[3])# [theta,phi,L] in radians
        pose_controller.goal_mode = data[0]
        if len(data)>5:
            extra1 = data[5]
            pose_controller.gcs_tether_tension = data[6]
            extra2 = data[7]
    return None

# Variable Initializations:
prev_yaw=0
prev_time = datetime.datetime.now()

# Modes that can be sent up by the GCS
GCS_mode = None
G_AUTO = 0
G_TAKEOFF = 1
G_LAND = 2

# Start the main loop
logging.info("------------------SYSTEM IS READY!!------------------")
logging.info("-----------------------------------------------------")

while True:

    t0= datetime.datetime.now()


    # Update State Information
    pose_controller.uav_coord = [autopilot.location.global_frame.lat, autopilot.location.global_frame.lon]     # GPS Coordinates of UAV [lat,lon] from pixhawk (DD.DDDDDDD)
    pose_controller.uav_vel = [autopilot.velocity[0],autopilot.velocity[1],autopilot.velocity[2]]      # UAV velocity [x,y,z] from pixhawk (m/s)
    pose_controller.uav_alt = (autopilot.location.global_relative_frame.alt )       # UAV Alt from pixhawk (m)
    pose_controller.uav_heading = autopilot.attitude.yaw        # UAV Heading (degrees)

    pose_controller.gcs_coord = [gcs.location.global_frame.lat, gcs.location.global_frame.lon]       # GPS Coordinates of GCS [lat,lon] from pixhawk (DD.DDDDDD)
    pose_controller.gcs_vel = [gcs.velocity[0], gcs.velocity[1], gcs.velocity[2]]        # GCS Velocity [x,y,z] from pixhawk (m/s)
    pose_controller.gcs_alt = (gcs.location.global_relative_frame.alt )         # GCS Altitude from pixhawk (m)
    pose_controller.gcs_heading = gcs.attitude.yaw       # GCS Heading (rad)
    
    print " ======== State Updated ========= "

    curr_time = datetime.datetime.now()
    delta = (curr_time-prev_time).total_seconds()
    if delta>2:
        read_mission()
        prev_time=curr_time
        #print "tried to read mission"
    

    #pose_controller.goal_pose = [data[0],data[1],data[2]] # UAV Goal Position [theta,phi,r] (radians)
    # pose_controller.goal_pose = [1.5,0,50]
    # output = pose_controller.run_pose_controller()
    # set_attitude_target(output)

###### CONTROLLER MANAGEMENT
# This set will only allow the control system to have control when both pixhawks are armed, and the UAV pixhawk is in GUIDED mode. 
# If the operator needs to recover the vehicle, he should change to ALTHOLD mode, and he will have full control of the vehicle. 
    if autopilot.mode.name=='GUIDED' and autopilot.armed and gcs.armed:

        if pose_controller.goal_mode ==G_AUTO:
            output = pose_controller.run_sph_pose_controller()
            set_attitude_target(output)
            condition_yaw(.5)

        if pose_controller.goal_mode ==G_TAKEOFF: 
            #Special condition for takoff. Positive pitch is pitch up
            roll = 0
            pitch = 0
            thr = .55
            output = pose_controller.special_att_control(roll,pitch,thr)
            set_attitude_target(output)
            print 'Take Off Mode. Roll: %.2f  Pitch: %.2f   Thr: %.2f'%(roll,pitch,thr)

        if pose_controller.goal_mode ==G_LAND:
            #Special condition for Landing. Positive pitch is pitch up
            roll = 0
            pitch = 0
            thr = .5
            output = pose_controller.special_att_control(roll,pitch,thr)
            set_attitude_target(output)
            print 'Land Mode. Roll: %.2f  Pitch: %.2f   Thr: %.2f'%(roll,pitch,thr)

        if pose_controller.goal_mode ==None:
            #Special condition for takoff. Positive pitch is pitch up
            roll = 0
            pitch = 0
            thr = .5
            output = pose_controller.special_att_control(roll,pitch,thr)
            set_attitude_target(output)
            print 'No Mode. Roll: %.2f  Pitch: %.2f   Thr: %.2f'%(roll,pitch,thr)
    else:
        print "Mode Not Guided: Manual Control"






    #Timing system to keep the control around CONTROL_DT
    t1 = datetime.datetime.now()
    dtc = (t1-t0).total_seconds()
    if dtc<CONTROL_DT:
        time.sleep(CONTROL_DT-dtc)
    else:
        print "Control Too Slow: ",dtc






    # #Get AirProbe Info:
    # try:
    #     air_probe_reading = data_from_airprobe.get(False)
    # except Empty:
    #     pass

    # # Get information from GCS. Update controller state information. 
    # try:
    #     gcs_state = DATA_FROM_GCS
    # except Empty:
    #     pass

    # pose_controller.gcs_tether_l = gcs_state[0]      # GCS Tether Length (m)
    # pose_controller.gcs_tether_tension = gcs_state[1] # GCS Tether Tension (newtons)

    # # Get information from GCS regarding status-- Mode and Goal Location
    # try:
    #     gcs_status = STATUS_FROM_GCS
    # except Empty:
    #     pass
    # gcs_mode = gcs_status[0]
    
    # # Operational and Mode Change Logic
    # if gcs_mode==G_AUTO:
    #     pose_controller.goal_pose = gcs_status[1]    # UAV Goal Position [theta,phi,r] (radians)

    # if gcs_mode==G_LAND:
    #     pose_controller.goal_pose = gcs_status[1]

    # if gcs_mode==G_TAKEOFF:
        
    #     #here is the actual arm and takeoff commands
    #     if gcs_mode_prev == None:
    #         while not autopilot.is_armable:
    #             print logging.info("Waiting for autopilot to be arm-able")
    #             time.sleep(1)
    #         autopilot.mode = VehicleMode("GUIDED")
    #         autopilot.armed=True
    #         while not autopilot.armed:
    #             logging.info("Waiting for autopilot to arm")
    #             time.sleep(.1)
    #         logging.info("Autpilot is Armed!!!")
    #     # Here is the takeoff sequence
    #     pose_controller.goal_pose = gcs_status[1]





    # if gcs_mode==None:
    #     #disarm vehicle. 

    # gcs_mode_prev = gcs_mode

    # control_outputs = pose_controller.run_pose_controller()











  
