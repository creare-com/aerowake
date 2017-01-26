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


gcs_connect_path = '/dev/ttyS0' #USe for RaspPi3
gcs_baud = 115200




#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! SYSTEM SETUP !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Local System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#### Logging Setup. ####

# Single Log Format:
# logging.basicConfig(filename='system.log',format='%(relativeCreated)s,%(levelname)s: %#(message)s',level=logging.DEBUG)

print("------------ STARTING AEROWAKE SYSTEM ------------")
print("-------------------- GCS NODE 0 ------------------")


####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pixhawk System Setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


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
    time.sleep(1)

    print "============="
    cmds=gcs.commands
    print cmds.download()
    cmds.clear()
    cmds.upload()



    











  
