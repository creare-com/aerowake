#Name: Main.py
#Author: mklinker
#Usage: Main function

import time
import datetime
import smbus
import numpy as np
from droneapi.lib import VehicleMode
from pymavlink import mavutil
import signal, sys
from Adafruit_ADS1x15 import ADS1x15

#Import Other Files
import Setup
import var
import LogData
import Failsafe

##############################################################
# Hardware Setup
##############################################################

print "Running Hardware Setup Routine"
adc0=Setup.InitializeADCHardware()
#api = local_connect()
#v=api.get_vehicles()[0]



##############################################################
# Main Loop
##############################################################

count=1
while count<10:
	Ch00 = adc0.readADCSingleEnded(0,var.adcGain,var.adcSPS)/1000
 	print "doink %.2f   %.2f" %(count,Ch00)
	LogData.writeToLogFile('DataPacket')
	count+=1
	time.sleep(.1)
