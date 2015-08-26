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
import imp

### Import Other Files

var = imp.load_source("var","../../../../home/pi/aerowake-mit/Controller/var.py")
Controller = imp.load_source("Controller","../../../../../../../../home/pi/aerowake-mit/Controller/Controller.py")
Setup = imp.load_source("Setup","../../../../home/pi/aerowake-mit/Controller/Setup.py")
LogData = imp.load_source("LogData","../../../../../../../../home/pi/aerowake-mit/Controller/LogData.py")

##############################################################
# Setup
##############################################################

print "Running Hardware Setup Routine"
adc0=Setup.InitializeADCHardware()
api = local_connect()
vehAPI=api.get_vehicles()[0]

#initialize Controller
print "Initialize Controller"
vehControl = Controller.Controller()

##############################################################
# DroneKit Specific Functions
##############################################################

def mavlinkCallback

#Update UAV Controller State
def updateUAV():
	print "Controller UAV State Updated"
	vehControl.uavAlt = vehAPI.location.alt
	vehControl.uavCoord = [vehAPI.location.lat,vehAPI.location.lon]
	vehControl.uavMode = vehAPI.mode.name
	vehControl.uavAttitude = [ vehAPI.attitude.roll , vehAPI.attitude.pitch ]
	vehControl.uavVel = [ vehAPI.velocity.vx, vehAPI.velocity.vy , vehAPI.velocity.vz ]
	
def updateShip():
	print "Controller Ship State Updated"
	vehControl.shipAlt = 0
	vehControl.shipCoord = [0,0]
	vehControl.ship.Heading = 0
	vehControl.tetherS = 0

def updateGoal(attitude,alt,angle)
	print "Controller Goal State Updated"
	vehControl.goalAttitude = attitude
	vehControl.goalAlt = alt
	vehControl.goalAngle = angle	

#Arm Vehicle
def armUAV():
	vehAPI.armed = True
	vehAPI.flush()
	while not vehAPI.armed and not api.exit:
		print "Waiting for UAV to arm..."
		time.sleep(1)

	vehControl.uavArmed = vehAPI.armed
	updateGoal([0,0],0,[0,0])
	print "Controller Status: %s" % vehAPI.armed

def disarmUAV():
	print "Disarming UAV"
	vehAPI.armed = False
	vehAPI.flush()
	print "Controller Status: %s" % vehAPI.armed
	vehControl.uavFlying = False
	vehControl.uavArmed = vehAPI.armed
	updateGoal([0,0],0,[0,0])


##############################################################
# Main Loop
##############################################################

#event: Takeoff Command Given

vehControl.goalAngle = [0,20]

##############################################################
# Mavlink Callback
##############################################################

#event: Wait at specified altitude until a command is given

#event: Command packet recieved, execute sweep

#event: land command given 

count=1
while count<10:
	updateUAV()
	Ch00 = adc0.readADCSingleEnded(0,var.adcGain,var.adcSPS)/1000
 	outData = "ran %.2f   %.2f" %(count,Ch00)
	print outData
	print "         altitude = " + str(vehAPI.location.alt)	
	LogData.writeToLogFile('outData')
	count+=1
	time.sleep(.1)






