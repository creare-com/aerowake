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
from reel.reel_main import reel_run


# Autopilot Connection Path. UDP for local simulation. 
autopilot_connect_path = 'udpin:0.0.0.0:14550'
#autopilot_connect_path = '/dev/ttyAMA0' #also set baud=57600
#autopilot_connect_path = '/dev/ttyUSB0'



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

path_prefix = "/media/pi/"
path_options = []
for s in glob.glob(path_prefix + "*"):
    path_options.append(s)
if(len(path_options) is 0):
    logging.critical("No external drive detected. Aborting.") 
    setup_abort()
    #sys.exit(1)
logging.info("External drive detected")

# Verify that the drive has 1GB min.
path = path_options[0]
statistics = os.statvfs(path)
free_space = statistics.f_bavail*statistics.f_bsize / (1024**3) #in Gb
if(free_space < 1):
    logging.critical("External drive has only %i GB free -- Please swap or clear external drive. Aborting." % free_space)
    setup_abort("External drive error")
    #sys.exit(1)
elif(free_space < 5):
    logging.warning("External drive has only %i GB free." % free_space)
else:
    logging.info("External drive has %i GB free." % free_space)

#### Create New Folder ####

flight_number = 1
while os.path.exists(path +"/flight_" + str(flight_number)):
    flight_number = flight_number + 1
flight_save_path = path + "/flight_" + str(flight_number)
os.makedirs(flight_save_path)
logging.info("Created new flight folder at: " + flight_save_path)
#### Start The CSV Log ####
csvwriter = csv.writer(open(str(flight_save_path)+"flight_log_" +str(datetime.datetime.now()) +".csv", 'wb'))



#### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Start Position Controller !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

pose_controller = pose_control()


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Multiprocessing System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


#### Start Air Probe Controller ####
commands_to_airprobe = Queue()
data_from_airprobe = Queue()
try:
    airprobe = airprobe_main(commands_to_airprobe, data_from_airprobe)
except Exception:
    logging.critical('Problem connection to airprobe. Aborting.')
    setup_abort("Airprobe System Failure")
    #sys.exit(1)
airporbe.start()


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### Pixhawk Connection ####
logging.info("Waiting for Pixhawk")
while True:
    try:
        #Note: connecting another GCS might mess up stream rates. Start mavproxy with --streamrate=-1 to leave stream params alone.
        autopilot = connect(autopilot_connect_path, heartbeat_timeout=60, rate=20, wait_ready=True)
        break
    except OSError:
        logging.critical("Cannot find device, is the Pixhawk plugged in? Retrying...")
        time.sleep(5)
    except APIException:
        logging.critical("Pixhawk connection timed out. Retrying...")
logging.info("Pixhawk connected!")

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


def abort_mission(reason):
    logging.critical('%s! Aborting mission.' % reason)
    send_msg_to_gcs(reason)
    autopilot.mode = VehicleMode("ALTHOLD")
    #TODO: TEST THIS OUT to make sure it doesnt lock us in whatever mode we specify. 

logging.info("------------------SYSTEM IS READY!!------------------")
logging.info("-----------------------------------------------------")

while True:

    #Get AirProbe Info:
    try:
        air_probe_reading = data_from_airprobe.get(False)
    except Empty:
        pass

    #Rest of the flight control setup. 




    control_outputs = pose_controller.run_pose_controller()











  
