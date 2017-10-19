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

import mission
from dronekit import APIException, VehicleMode, connect, mavutil, Command
from reel.reel import reel_run
from interface.interface import interface_run


 #!# All comments to explain system will be prefaced with "#!#"

 #!# Setting up connection path for the Autopilots. 
#gcs_connect_path = '/dev/ttyAMA0' #Choose which ever is applicable
gcs_connect_path = '/dev/ttyS0' #For the RaspPi3 
#gcs_connect_path = '/dev/ttyUSB0'
#gcs_connect_path = '/dev/ttyACM0'
gcs_baud = 115200

if len(sys.argv) >= 2 and sys.argv[1].startswith('sim'):
     #!# For SITL testing, use the following. The UAV is located on Port 14552 and GCS 14554
    gcs_connect_path = '127.0.0.1:14556'
    gcs_baud = 115200
 
if __name__ == '__main__':

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
    logging.getLogger('Adafruit_I2C').setLevel(logging.ERROR)




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
#    time.sleep(2)
    #### External Drive Setup ####


    ####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Multiprocessing System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

     #!# This block will start both the interface and the reel controller. 
    #!# Both of these systems use the multiprocessing infrastructure. 

    status_to_interface = Queue()
    data_from_interface = Queue()

    try:
        ui = interface_run(status_to_interface,data_from_interface)
    except Exception:
        logging.critical('Problem connecting to UI')
        setup_abort("UI Failure")
    ui.start()


    #### Start Reel Controller ####
    commands_to_reel = Queue()
    data_from_reel = Queue()
    try:
        reel = reel_run(commands_to_reel, data_from_reel)
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

    last_heartbeat_dt = 0
    @gcs.on_attribute('last_heartbeat')   
    def last_heartbeat_listener(self, attr_name, value):
        if(attr_name is 'last_heartbeat'):
            global last_heartbeat_dt
            last_heartbeat_dt = value

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
        # Command the tether to spool out/reel in to the appropriate length
        commands_to_reel.put({"cmd":"goto", "L":L})

        try:
            # Store waypoints in GCS PixHawk.  The UAV Pi will read them out and maneuver the UAV to that position.
            # Waypoints here are not in a format that ether PixHawk can usefully interpret directly (theta, phi, L).
            cmds = gcs.commands
            # I don't know why Mike put this in here.  It could be important.
            # But for now I'm just gonna comment it out.  -JDW
            # logging.info("Downloading current waypoints...")
            # cmds.download()
            # cmds.wait_ready()
            # logging.info("Done downloading.")
            cmds.clear()
            cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, mode, 0, 0, 0, theta, phi, L)
            cmd2=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, -1,  0, 0, 0, extra1, tether_t, extra2)
            cmds.add(cmd1)
            cmds.add(cmd2)
            logging.info("Uploading new waypoints...")
            cmds.upload() # Asynchronous
            cmds.wait_ready() # Make it synchronous
            logging.info("Done uploading.")
        except dronekit.APIException as err:
            logging.error("Couldn't set waypoint on GCS: " + str(err))
        
    def print_mission():
        missionlist = download_mission()
        for cmd in missionlist:
            #(cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
            print cmd.param1
            print cmd.x
            print cmd.y
            print cmd.z

    def clear_mission():
        cmds = gcs.commands
        cmds.download()
        cmds.wait_ready()
        cmds.clear()
        cmds.upload()

    #!# At this point, arm the GCS pixhawk and place it into guided mode. 
    #1# Might have to set the ARMING CHECK parameter to 0 in the GCS pixhawk. 

    gcs.channels.overrides = {'1':1500, '2':1500, '3':1000, '4':1500, '5':1500,'6':1500}

    gcs.mode = VehicleMode("GUIDED")

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
    #!# waypoint_num=0 is the index for the mission. 
    GCS_cmd = None
    G_AUTO = 0
    G_TAKEOFF = 1
    G_LAND = 2
    mode=None
    waypoint_num=0
    phi   = '-'
    theta = '-'
    L     = '-'

    #!# Start the main loop. 
    try:
        run = True
        reel_reading = {'L':'-', 'T':'-'}
        while run:
            display_vars = {}
            time.sleep(.2)
            gcs.mode = VehicleMode('GUIDED')
            gcs.armed=True
            # print "Run"
            #Get Reel Info:
            try:
                 # Expected to return {"L": <length in meters, as double>, "T": <tension in newtons, as double>}
                 reel_reading = data_from_reel.get(False)
                 # print("Got reel data:" + str(reel_reading))
                 # Currently not used anywhere
            except Empty:
                 pass

            # # Determine flight mode and waypoints for the vehicle

            if gcs.armed:
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
                    waypoint_num=0
                    phi = mission.PHI[waypoint_num]
                    theta = mission.THETA[waypoint_num]
                    L = mission.L[waypoint_num]
                    mode = G_AUTO
                    set_waypoint(mode,theta,phi,L)
                    print "Entered Auto Mode"
                    print "Mode: ",mode," Theta: %.1f  Phi: %.1f  L: %.1f" %(mission.THETA[waypoint_num],mission.PHI[waypoint_num],mission.L[waypoint_num])
                    print_mission()

                elif GCS_cmd == "ADV_CMD":
                    waypoint_num+=1
                    if waypoint_num==len(mission.THETA):
                        waypoint_num=0

                    phi = mission.PHI[waypoint_num]
                    theta = mission.THETA[waypoint_num]
                    L = mission.L[waypoint_num]
                    mode = G_AUTO
                    set_waypoint(mode,theta,phi,L)
                    GCS_cmd = "AUTO_CMD"
                    print "Advanced Goal Target"
                    print "Mode: ",mode," Theta: %.1f  Phi: %.1f  L: %.1f" %(mission.THETA[waypoint_num],mission.PHI[waypoint_num],mission.L[waypoint_num])
                    print_mission()

                elif GCS_cmd == "DEC_CMD":
                    waypoint_num -= 1
                    if waypoint_num < 0:
                        waypoint_num = 0

                    phi = mission.PHI[waypoint_num]
                    theta = mission.THETA[waypoint_num]
                    L = mission.L[waypoint_num]
                    mode = G_AUTO
                    set_waypoint(mode,theta,phi,L)
                    GCS_cmd = "AUTO_CMD"
                    print "Decremented Goal Target"
                    print "Mode: ",mode," Theta: %.1f  Phi: %.1f  L: %.1f" %(mission.THETA[waypoint_num],mission.PHI[waypoint_num],mission.L[waypoint_num])
                    print_mission()

                elif GCS_cmd == "RESEND_CMD":
                    phi = mission.PHI[waypoint_num]
                    theta = mission.THETA[waypoint_num]
                    L = mission.L[waypoint_num]
                    mode = G_AUTO
                    set_waypoint(mode,theta,phi,L)
                    GCS_cmd = "AUTO_CMD"
                    print "Resent Goal Target"
                    print "Mode: ",mode," Theta: %.1f  Phi: %.1f  L: %.1f" %(mission.THETA[waypoint_num],mission.PHI[waypoint_num],mission.L[waypoint_num])
                    print_mission()


                elif GCS_cmd == "TAKEOFF_CMD":
                    phi = 0
                    theta = 1.5
                    L = 20
                    mode = G_TAKEOFF
                    set_waypoint(mode,theta,phi,L)
                    print "Takeoff Waypoint Set"
                    print "Mode: ",mode
                    print_mission()

                elif GCS_cmd == "LAND_CMD":
                    phi   = 0
                    theta = 1.5
                    L     = 20
                    mode = G_LAND
                    set_waypoint(mode,theta,phi,L)
                    print "Land Waypoint Set"
                    print "Mode: ",mode
                    print_mission()

                elif GCS_cmd == "QUIT_CMD":
                    run = False # Cleanup tasks handled after main loop
                
                elif GCS_cmd == "HALT_REEL_CMD":
                    commands_to_reel.put({"cmd":"halt"})

                GCS_cmd="WAITING"

            else:
                print "GCS Not Armed"
                
            # Update the UI
            mode_string = "Auto position" if mode == G_AUTO else \
                          "Takeoff"       if mode == G_TAKEOFF else \
                          "Landing"       if mode == G_LAND else \
                          "Invalid state"
            display_vars = [
                ("GCS PH heartbeat time",    last_heartbeat_dt),
                ("System mode",              mode_string      ),
                ("Waypoint number",          waypoint_num     ),
                ("Target Phi (Az, rad)",     phi              ),
                ("Target Theta (el, rad)",   theta            ),
                ("Target L (m)",             L                ),
                ("Current reel L (m)",       reel_reading['L']),
                ("Current reel tension (N)", reel_reading['T']),
            ]
            status_to_interface.put(display_vars)

    except KeyboardInterrupt:
        print("Got ^C, Cleaning up")

    # A clean exit stops the reel prior to exiting.
    # An unclean exit might leave the reel moving.
    commands_to_reel.put({"cmd":"exit"})
    reel.join()
    if ui.is_alive(): # under a ctrl-c, this will be oblivious
        ui.terminate()
        
