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

var = imp.load_source("var","../../../../home/pi/aerowake-mit/Controller/var.py")
Controller = imp.load_source("Controller","../../../../../../../../home/pi/aerowake-mit/Controller/Controller.py")
Setup = imp.load_source("Setup","../../../../home/pi/aerowake-mit/Controller/Setup.py")
LogData = imp.load_source("LogData","../../../../../../../../home/pi/aerowake-mit/Controller/LogData.py")

##############################################################
# Setup
##############################################################

AdcEnable=False #make True if you have the ADS1015 connected via i2c

print "\n \n \n \n \n \n -- AeroWake Controller \n \n \n"
print " -- Running Hardware Setup Routine"
if AdcEnable:
	adc0=Setup.InitializeADCHardware()
api = local_connect()
vehAPI=api.get_vehicles()[0]

#initialize Controller
print " -- Initialize Controller"
vehControl = Controller.Controller()

##############################################################
# DroneKit Specific Functions
##############################################################


#Update UAV Controller State
def updateUAV():
	#print " -- Controller UAV State Updated"
	vehControl.uavAlt = vehAPI.location.alt
	vehControl.uavCoord = [vehAPI.location.lat,vehAPI.location.lon]
	vehControl.uavMode = vehAPI.mode.name
	vehControl.uavAttitude = [ vehAPI.attitude.roll , vehAPI.attitude.pitch ]
	vehControl.uavVel = vehAPI.velocity
	
def updateShip():
	print " -- Controller Ship State Updated"
	vehControl.shipAlt = 0
	vehControl.shipCoord = [0,0]
	vehControl.ship.Heading = 0
	vehControl.tetherS = 0

def updateGoal(attitude,alt,angle):
	print " -- Controller Goal State Updated"
	vehControl.goalAttitude = attitude
	vehControl.goalAlt = alt
	vehControl.goalAngle = angle	

#Arm Vehicle
def armUAV():
	print "\n  -- Arming UAV \n"
	vehAPI.channel_override = { "3" : 1000}
	vehAPI.flush()
	vehAPI.armed = True
	vehAPI.flush()
	while not vehAPI.armed and not api.exit:
		print " -- Waiting for UAV to arm..."
		time.sleep(.5)

	vehControl.uavArmed = vehAPI.armed
	updateGoal([0,0],0,[0,0])
	print " -- Controller Status: Vehicle Armed: %s" % vehAPI.armed

def disarmUAV():
	print "\n  -- Disarming UAV \n"
	updateGoal([0,0],0,[0,0])
	vehAPI.channel_override = {"3" : 999 }
	vehAPI.flush()
	vehAPI.armed = False
	vehAPI.flush()
	while vehAPI.armed and not api.exit:
		print " -- Waiting for UAV to disarm..."	
		time.sleep(.5)

	print " -- Controller Status: Vehicle Armed: %s" % vehAPI.armed
	vehControl.uavFlying = False
	vehControl.uavArmed = vehAPI.armed

def takeOffUAV(alt):
	#Usage: Take the uav off and hover at [0,0] attitude at the specified altitude. 
	if vehAPI.armed and not vehControl.uavFlying and not vehControl.uavFailsafe:
		vehControl.uavFlying = True
		updateGoal([0,0],alt,[0,45])
		#Some machinery to increase the throttle and close the loop around a desired altitude. 

def landUAV(landAlt):
	#Usage: Return the UAV to hovering close to the ship. Following that, the Uav can be reeled in, and the Uav disarm can be triggered 
	if vehAPI.armed and vehControl.uavFlying and not vehControl.uavFailsafe:
		updateGoal([0,0],landAlt,[0,45])
		# At this point, need to reel in the tether, then trigger UAV Disarm

##############################################################
# Mavlink Callback
##############################################################

#Setup Attitude Callback
def uav_callback(attitude):
	#print "UAV Callback" 
	updateUAV()	
	vehControl.runController()	

vehAPI.add_attribute_observer('attitude' , uav_callback)

def ship_callback():
	print " -- Ship Callback"


##############################################################
# Main Loop
##############################################################

#event: Takeoff Command Given

#event: Wait at specified altitude until a command is given

#event: Command packet recieved, execute sweep

#event: land command given

armUAV()

# Run this for a bit as a test
count=1
while count<50:
	if AdcEnable:
		Ch00 = adc0.readADCSingleEnded(0,var.adcGain,var.adcSPS)/1000
	else:
		Ch00=0.00 	

	roll = float(vehControl.uavAttitude[0])
	pitch = float(vehControl.uavAttitude[1])
	outData = " -- Count: %.2f    ADC: %.2f   Roll: %.2f  Pitch: %.2f" %(count,Ch00,roll,pitch)
	print outData
	#print "         altitude = " + str(vehAPI.location.alt)	
	#LogData.writeToLogFile('outData')
	count+=1
	time.sleep(.1)


disarmUAV()






