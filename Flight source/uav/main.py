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
import numpy as np
from multiprocessing import Queue
from Queue import Empty
from dronekit import APIException, VehicleMode, connect, mavutil
from airprobe.airprobe_run import airprobe_run

 #!# Comments explaining system prefaced with "#!#" - generally higher level understanding of process.
 #!# Comments explaining code prefaced with '#'

 #!# Select which position controller to use. pose_control is the spherical position controller and pose_control_cart is the Cartesian based system. 

#from controller.pose_control_cart import pose_controller_class
from controller.pose_control import pose_controller_class


#######################################################
##
##               Function definitions
##
#######################################################

 #!# This function is called if there is a problem with the vehicle setup. It will kill the script. 

def setup_abort(abort_reason=None):
    t = 0
    #display items on screen
    while t<10:
        logging.critical("System aborting due to error: %s" %abort_reason) 
        time.sleep(1)
        t+=1
    sys.exit(1)

 #!# This is a functions that is part of DroneKit but is currently not used. 
#def send_msg_to_gcs(message):
#    msg = autopilot.message_factory.statustext_encode(mavutil.mavlink.MAV_SEVERITY_CRITICAL, message)
#    autopilot.send_mavlink(msg)
#    autopilot.flush()

 #!# This Abort Mission function an be called ONCE in the event that the mission should be autonmously terminated. It is important to NOT call this function multiple times or every control loop, as it will lock the system into Altitude Hold mode, and the UAV operator might not be able to override it. 
def abort_mission(reason):
    global autopilot
    logging.critical('%s! Aborting mission.' % reason)
    autopilot.mode = VehicleMode("ALTHOLD")
    #TODO: TEST THIS OUT to make sure it doesnt lock us in whatever mode we specify. 

 #!#  This function is very important and is used to set the attitude of the vehicle. The bitmask is set here to take in a quaternion and throttle setting from 0-1. This throttle setting is that same as altitude hold mode. A value of .5 will maintain altitude. 
def set_attitude_target(data_in):
    global autopilot

    if np.size(data_in) == 6:
        # Then in G_AUTO mode and commanding yaw
        yaw_rate = data_in[5]
    else:
        # Then in G_TAKEOFF or G_LAND or None mode and not commanding yaw
        yaw_rate = 0

    quat = data_in[0:4]
    thr = data_in[4]
    msg = autopilot.message_factory.set_attitude_target_encode(
        0, 0,0,
        0b00000001,    # bitmask
        # 0b000000001,    # bitmask
        quat,           # quat
        0,              #roll rate
        0,              #  pitch rate
        # 0,       # yaw speed rad/s
        yaw_rate,       # yaw speed rad/s
        thr)            # thrust
    autopilot.send_mavlink(msg)

def send_global_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = autopilot.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    autopilot.send_mavlink(msg)

 #!# This function is a legacy item, but will allow the system to commmand a yaw heading. 
def condition_yaw(heading, is_relative=False):
    global autopilot

    print '\n\n\nCONDITIONING YAW TO HEADING OF: ', heading,'\n\n\n'

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

 #!#  This function downloads all of the current waypoints from the GCS pixhawk (if there are any), clears the GCS pixhawk mission, and returns to mission information. 
def download_mission():
    global autopilot
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

 #!#  This function takes the mission list from the  GCS pixhawk, and parses it into a command. This system uses 2 waypoints to specify a command from the ground station. 
 #!#  First waypoint holds goal and mode information. Second waypoint holds tether information, and there is room for two extra parameters. 
def read_mission():
    global autopilot
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
    return pose_controller.goal_mode


#######################################################
##
##               Main
##
#######################################################

if __name__ == '__main__':

     #!# Setting up connection path for the Autopilots. 
     #!# For Hardware operation, use the following. These baud rates must match those
     #!# as established on the actual hardware. UAV wired connection should be 115200 and the telemetry radio should be set up for 57600. Uncomment the following lines:

    #autopilot_connect_path = '/dev/ttyAMA0'
    autopilot_connect_path = '/dev/ttyS0' #USe for RaspPi3
    uav_baud = 115200
    gcs_connect_path = '/dev/ttyUSB0'
    gcs_baud = 57600

    if len(sys.argv) >= 2 and sys.argv[1].startswith('sim'):
         #!# For SITL testing, use the following. The UAV is located on Port 14552 and GCS 14554
        autopilot_connect_path = '127.0.0.1:14552'
        gcs_connect_path = '127.0.0.1:14554'
        uav_baud = 115200
        gcs_baud = 115200


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


     #!# This prints out saying that we are starting the system initialization process. 
    print "\n\n\n\n"
    logging.info("------------ STARTING AEROWAKE SYSTEM ------------")
    logging.info("-------------------- UAV NODE --------------------")
    #### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Start Position Controller !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

     #!# This establishes the position control system. 
     #!# CONTROL_DT is the control loop speed and needed for calculating the control derivative terms. This parameter might have to be updated each loop iteration, or else the derivative control term might give random large inputs when the control loop slows for random reasons. 
    CONTROL_DT = .1
    pose_controller = pose_controller_class(CONTROL_DT)
    pose_controller.log_file_name = time.strftime('telem_log_%Y-%m-%d_%H%M%S.csv',time.localtime())
    #pose_controller.run_sph_pose_controller()

    ####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Multiprocessing System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

     #!# This below will begin the Airprobe control system. TODO: Implement this. 

    #### Start Air Probe Controller ####
    commands_to_airprobe = Queue()
    data_from_airprobe = Queue()
    try:
        airprobe = airprobe_run(commands_to_airprobe, data_from_airprobe)
    except Exception:
        logging.critical('Problem connection to airprobe. Aborting.')
        setup_abort("Airprobe System Failure")
        sys.exit(1)
    airprobe.start()

    ####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

     #!# This block will connect to the two different pixhawks. It will connect to the UAV pixhawk first, 
     #!# followed by the GCS pixhawk. Each autopilot will be given 60 seconds to connect. 
     #!# If either connection fails, the script will exit. During connection the Pixhawks should be powered on, 
     #!# initialized, and blinking green or yellow. If the script continually fails, reboot the pixhawks and check the paths. 
     #!# If the GCS continually fails, check the telemetry radios. If the green lights on the telemetry radios
     #!# are blinking, that indicates that the radios are not properly communicating. A solid green light on the
     #!# telemetry radios indicates they are connected. 


    #### Autopilot Connection ####
    logging.info("Waiting for Autopilot")
    while True:
        try:
            #Note: connecting another GCS might mess up stream rates. Start mavproxy with --streamrate=-1 to leave stream params alone.
            autopilot = connect(autopilot_connect_path,baud=uav_baud, heartbeat_timeout=60, rate=20, wait_ready=True)
            break
        except OSError:
            logging.critical("Cannot find device, is the Autopilot plugged in? Retrying...")
            time.sleep(5)
        except APIException:
            logging.critical("Autopilot connection timed out. Retrying...")
    logging.info("Autopilot connected!")

    if(autopilot.parameters['ARMING_CHECK'] != 1):
        logging.warning("Autopilot reports arming checks are not standard!")

    print "AUTOPILOT PIXHAWK IS CONNECTED!!!!"

    #### GCS Connection ####
    logging.info("Waiting for GCS")
    while True:
        try:
            gcs = connect(gcs_connect_path,baud=gcs_baud,heartbeat_timeout=60, rate=20, wait_ready=True)
            break
        except OSError:
            logging.critical("Cannot find device, is the GCS connected? Retrying...")
            time.sleep(5)
        except APIException:
            logging.critical("GCS connection timed out. Retrying...")
    logging.info("GCS Connected")


    print "GCS PIXHAWK IS CONNECTED!!!!!!!"


    #### System Time Setup ####
    logging_time=0
    # We need to get the time from the autopilot (which gets it via gps), because the raspi does not have a RTC
    autopilot_start_time = 0
    rasp_start_time = 0
    @autopilot.on_message('SYSTEM_TIME')
    def autopilot_time_callback(self, attr_name, msg):
        global autopilot_start_time
        if(autopilot_start_time is 0 and msg.time_unix_usec > 0):
            autopilot_start_time = msg.time_unix_usec/1000000
            # logging_time = datetime.datetime.fromtimestamp(autopilot_start_time).strftime('%Y-%m-%d %H:%M:%S')
            logging_time = "%0.4f" % time.time()
            logging.info("Got GPS lock at %s" % logging_time)
            rasp_start_time = time.clock()


    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk Callback/Logging System !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

     #!# Log if no MAVLink packets are heard for more than a second
     #!# Also log mode changes, and arm/disarm
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

     #!# Lastly, various variable initializations. 

    # Variable Initializations:
    prev_yaw=0
    yaw_editable = False
    prev_time = datetime.datetime.now()

     #!# These are the Modes that can be sent up by the GCS
    GCS_mode = None
    G_AUTO = 0
    G_TAKEOFF = 1
    G_LAND = 2

     #!#  Start the main loop
    logging.info("------------------SYSTEM IS READY!!------------------")
    logging.info("-----------------------------------------------------")

    if not autopilot.mode.name == "GUIDED":
        autopilot.mode = VehicleMode("GUIDED")
        while not autopilot.mode.name == "GUIDED":
            print 'Waiting for UAV to sqitch to GUIDED mode: ', autopilot.mode
            time.sleep(1)

    if not autopilot.armed:
        while not autopilot.is_armable:
            print "UAV Not Armable"
            time.sleep(.5)
        autopilot.armed = True
        while not autopilot.armed:
            print 'Waiting for UAV to arm: ',autopilot.armed
            time.sleep(0.5)

    run = True
    while run:


        t0= datetime.datetime.now()
        pose_controller.logging_time = logging_time

         #!#  Update State Information from the UAV pixhawk and GCS pixhawk. 
        pose_controller.uav_coord = [autopilot.location.global_frame.lat, autopilot.location.global_frame.lon]     # GPS Coordinates of UAV [lat,lon] from pixhawk (DD.DDDDDDD)
        pose_controller.uav_vel = [autopilot.velocity[0],autopilot.velocity[1],autopilot.velocity[2]]      # UAV velocity [x,y,z] from pixhawk (m/s)
        pose_controller.uav_alt = (autopilot.location.global_relative_frame.alt )       # UAV Alt from pixhawk (m)
        pose_controller.uav_heading = autopilot.attitude.yaw        # UAV Heading (rad)
        pose_controller.uav_voltage = autopilot.battery.voltage     # UAV Voltage
        pose_controller.uav_current = autopilot.battery.current/10 # UAV Current in mA

        pose_controller.gcs_coord = [gcs.location.global_frame.lat, gcs.location.global_frame.lon]       # GPS Coordinates of GCS [lat,lon] from pixhawk (DD.DDDDDD)
        pose_controller.gcs_vel = [gcs.velocity[0], gcs.velocity[1], gcs.velocity[2]]        # GCS Velocity [x,y,z] from pixhawk (m/s)
        pose_controller.gcs_alt = (gcs.location.global_relative_frame.alt )         # GCS Altitude from pixhawk (m)
        pose_controller.gcs_heading = gcs.attitude.yaw       # GCS Heading (rad)
        
        # print " ======== State Updated ========= "


        #!# This block will read try to read any new mission commands from the GCS. 
        #!# If there is a new set of commands, read_mission() will read, parse, and clear the mission. 
        curr_time = datetime.datetime.now()
        delta = (curr_time-prev_time).total_seconds()
        if delta>1:
            read_mission()
            prev_time=curr_time
            #print "tried to read mission"

        #!# This block will read the Airprobe information from the multiprocessing queue. 
        # #Get AirProbe Info:
        # try:
        #     air_probe_reading = data_from_airprobe.get(False)
        # except Empty:
        #     pass

        #!# ###### CONTROLLER MANAGEMENT
        #!#  This set will only allow the control system to have control when both pixhawks are armed, and the UAV pixhawk is in GUIDED mode. 
        #!#  If the operator needs to recover the vehicle, he should change to ALTHOLD mode, and he will have full control of the vehicle. 

        # print "UAV mode: " + autopilot.mode.name + " Armed? " + str(autopilot.armed)
        # print "GCS mode: " + gcs.mode.name + " Armed? " + str(gcs.armed)

        #if autopilot.mode.name=='GUIDED' and autopilot.armed and gcs.armed and run:
        if True and run:
            # pose_controller.goal_mode=G_AUTO

            # if not yaw_editable:
            #     heading_orig = autopilot.attitude.yaw*180/np.pi # degrees
            #     while not yaw_editable:
            #         send_global_velocity(0, 0, 0)
            #         time.sleep(1)
            #         print 'Forcing yaw to be editable'
            #         roll_cmd = 0
            #         pitch_cmd = 0
            #         yaw_cmd = 270*np.pi/180
            #         thr_cmd = 0.75
            #         tol = 0.001
            #         quat = pose_controller.eul2quat(roll_cmd,pitch_cmd,yaw_cmd)
            #         output = [quat[0],quat[1],quat[2],quat[3],thr_cmd,yaw_cmd]
            #         set_attitude_target(output[0:5])
            #         heading_desired_absolute = yaw_cmd*180/np.pi # degrees
            #         condition_yaw(heading_desired_absolute)
            #         # Send position (in this case velocity) command to allow the condition_yaw() function to work
            #         time.sleep(2)
            #         print '\n\n\n',autopilot.attitude.yaw, (autopilot.attitude.yaw - yaw_cmd*np.pi/180)**2,'\n\n\n'
            #         if autopilot.attitude.yaw**2 - yaw_cmd**2 < tol:
            #             yaw_editable = True

            uav_heading = pose_controller.uav_heading
            gcs_heading = pose_controller.gcs_heading
            uav_to_gcs_bearing = pose_controller.get_bearing()
            relative_yaw_cmd = uav_to_gcs_bearing - uav_heading
            if relative_yaw_cmd < -np.pi:
                relative_yaw_cmd = relative_yaw_cmd + 2*np.pi
            elif relative_yaw_cmd > np.pi:
                relative_yaw_cmd = relative_yaw_cmd - 2*np.pi

            print '\n\n\n'
            print 'UAV'
            print '\theading: ',uav_heading*180/np.pi
            # print '\tcoord: ',pose_controller.uav_coord
            # print '\talt: ',pose_controller.uav_alt
            # print '\nGCS'
            # print '\theading: ',gcs_heading*180/np.pi
            # print '\tcoord: ',pose_controller.gcs_coord
            # print '\talt: ',pose_controller.gcs_alt
            print '\nBEARING (uav2gcs): ', uav_to_gcs_bearing*180/np.pi
            print '\Yaw Rate Command (CW): ', relative_yaw_cmd*180/np.pi
            print '\n\n\n'

            if pose_controller.goal_mode == G_AUTO:
                output = pose_controller.run_sph_pose_controller()
                set_attitude_target(output)

            if pose_controller.goal_mode == G_TAKEOFF: 
                #Special condition for takoff. Positive pitch is pitch up
                roll = 0
                pitch = 0.25
                thr = .6
                output = pose_controller.special_att_control(roll,pitch,thr)
                set_attitude_target(output)
                print 'Take Off Mode. Roll: %.2f  Pitch: %.2f   Thr: %.2f'%(roll,pitch,thr)

            if pose_controller.goal_mode == G_LAND:
                #Special condition for Landing. Positive pitch is pitch up
                roll = 0
                pitch = 0
                thr = 0.1
                output = pose_controller.special_att_control(roll,pitch,thr)
                set_attitude_target(output)
                print 'Land Mode. Roll: %.2f  Pitch: %.2f   Thr: %.2f'%(roll,pitch,thr)

            if pose_controller.goal_mode == None:
                #Special condition for takoff. Positive pitch is pitch up
                roll = 0
                pitch = 0
                thr = .5
                output = pose_controller.special_att_control(roll,pitch,thr)
                set_attitude_target(output)
                print 'No Mode. Roll: %.2f  Pitch: %.2f   Thr: %.2f'%(roll,pitch,thr)
        else:
            print "Mode Not Guided: Manual Control"

         #!# Timing system to keep the control around CONTROL_DT
         #!# If the script is consistently too slow, the Control DT will have to be updated every loop iteration
        t1 = datetime.datetime.now()
        dtc = (t1-t0).total_seconds()
        if dtc<CONTROL_DT:
            time.sleep(CONTROL_DT-dtc)
        else:
            print "Control Too Slow: ",dtc

        










      
