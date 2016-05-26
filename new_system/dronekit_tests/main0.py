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
#autopilot_connect_path = 'udpin:0.0.0.0:14550'
#autopilot_connect_path = '/dev/ttyAMA0' #also set baud=57600
#autopilot_connect_path = '/dev/ttyUSB0'


autopilot_connect_path = '/dev/ttyS0' #USe for RaspPi3
uav_baud = 115200
gcs_connect_path = '/dev/ttyUSB0'
gcs_baud = 57600



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




while True:
    time.sleep(.1)

    print "============="
    print autopilot.attitude
    print gcs.attitude



    #print "Global Location: %s" %autopilot.location.global_frame.lat







  
