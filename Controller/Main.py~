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

time.sleep(1)

AdcEnable=False #make True if you have the ADS1015 connected via i2c

print "\n \n \n \n \n \n -- AeroWake Controller \n \n \n"
print " -- Running Hardware Setup Routine"
if AdcEnable:
    adc0=Setup.InitializeADCHardware()

## Connection to API
#api = local_connect()
#vehAPI=api.get_vehicles()[0]

api,gcs = local_connect()
vehAPI = api.get_vehicles()[0]


#initialize Controller
print " -- Initialize Controller"
vehControl = Controller.Controller()

##############################################################
# DroneKit Specific Functions
##############################################################


#Update UAV Controller State
def update_uav():
    #print " -- Controller UAV State Updated"
    vehControl.uav_alt = vehAPI.location.alt
    vehControl.uav_coord = [vehAPI.location.lat,vehAPI.location.lon]
    vehControl.uav_mode = vehAPI.mode.name
    vehControl.uav_attitude = [ vehAPI.attitude.roll , vehAPI.attitude.pitch ]
    vehControl.uav_vel = vehAPI.velocity
    vehControl.uav_heading = vehAPI.attitude.yaw  #check this
        

def update_ship():
    print " -- Controller Ship State Updated"
    vehControl.ship_alt = 0
    vehControl.ship_coord = [0,0]
    vehControl.ship_heading = 0
    vehControl.ship_tether_length = 0

def update_goal(attitude,alt,angle):
    print " -- Controller Goal State Updated"
    vehControl.goal_attitude = attitude
    vehControl.goal_alt = alt
    vehControl.goal_angle = angle   

#Arm Vehicle
def arm_UAV():
    print "\n  -- Arming UAV \n"
    vehAPI.channel_override = { "3" : 1000}
    vehAPI.flush()
    vehAPI.armed = True
    vehAPI.flush()
    while not vehAPI.armed and not api.exit:
        print " -- Waiting for UAV to arm..."
        time.sleep(.5)

    vehControl.uav_armed = vehAPI.armed
    update_goal([0,0],0,[0,0])
    print " -- Controller Status: Vehicle Armed: %s" % vehAPI.armed

def disarm_UAV():
    print "\n  -- Disarming UAV \n"
    update_goal([0,0],0,[0,0])
    #vehAPI.channel_override = {"3" : 999 }
    vehAPI.flush()
    vehAPI.armed = False
    vehAPI.flush()
    while vehAPI.armed and not api.exit:
        print " -- Waiting for UAV to disarm..."    
        time.sleep(.5)

    print " -- Controller Status: Vehicle Armed: %s" % vehAPI.armed
    vehControl.uav_flying = False
    vehControl.uav_armed = vehAPI.armed

def takeoff_UAV(alt):
    #Usage: Take the uav off and hover at [0,0] attitude at the specified altitude. 
    if vehAPI.armed and not vehControl.uav_flying and not vehControl.uav_failsafe:
        vehControl.uav_flying = True
        update_goal([0,0],alt,[0,45])
        #Some machinery to increase the throttle and close the loop around a desired altitude. 

def land_UAV(landAlt):
    #Usage: Return the UAV to hovering close to the ship. Following that, the Uav can be reeled in, and the Uav disarm can be triggered 
    if vehAPI.armed and vehControl.uav_flying and not vehControl.uav_failsafe:
        update_goal([0,0],landAlt,[0,45])
        # At this point, need to reel in the tether, then trigger UAV Disarm

def human_rc_control():
    #usage: Call this to give control back to the humans RC Controller
    vehControl.rc_write_priv = False
    vehAPI.channel_override = { "1":0, "2":0, "3":0, "4":0, "5":0, "6":0, "7":0, "8":0}
    vehAPI.flush()
    
def controller_rc_control():
    #usage: Reset flag to give the controller write-privileges to the RC Override. 
    vehControl.rc_write_priv = True

def control_check():
    if vehControl.uav_mode != 'STABILIZE':
        human_rc_control()
    if vehControl.uav_mode == 'STABILIZE' and not vehControl.rc_write_priv:
        controller_rc_control()

def write_channels(ch_out):
    if vehControl.rc_write_priv:
        vehAPI.channel_override = {"1":ch_out[0], "2":ch_out[1], "3":ch_out[2], "4":ch_out[3]}
        vehAPI.flush()
        print "---- Controller Wrote to Channels"
    else:   
        print "---- RC Control Active"     

##############################################################
# Mavlink Callback
##############################################################

#Setup Attitude Callback
def uav_callback(attitude):
    #usage: This will update the UAV state in the controller whenever it receives a new packet from the telemetry link over ttyAMA0
    #print "UAV Callback" 
    update_uav()    
    control_check()
    channel_out = vehControl.run_controller() 
    write_channels(channel_out)


    
vehAPI.add_attribute_observer('attitude' , uav_callback)

def ship_callback():
    #usage: This will update the ship state and the goal state when it recieves a new packet from the GCS. 
    update_ship()
    update_goal()
    print " -- Ship Callback"


##############################################################
# Main Loop
##############################################################
human_rc_control()

#event: Takeoff Command Given

#event: Wait at specified altitude until a command is given

#event: Command packet recieved, execute sweep

#event: land command given

arm_UAV()

controller_rc_control()
# Run this for a bit as a test
count=1
while count<50:
    if AdcEnable:
        Ch00 = adc0.readADCSingleEnded(0,var.adcGain,var.adcSPS)/1000
    else:
        Ch00=0.00   

    roll = float(vehControl.uav_attitude[0])
    pitch = float(vehControl.uav_attitude[1])
    outData = " -- Count: %.2f    ADC: %.2f   Roll: %.2f  Pitch: %.2f" %(count,Ch00,roll,pitch)
    #print outData
    #print "         altitude = " + str(vehAPI.location.alt)    
    #LogData.writeToLogFile('outData')
    count+=1
    time.sleep(.2)


human_rc_control()
disarm_UAV()






