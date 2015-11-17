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
#from Adafruit_ADS1x15 import ADS1x15
import imp

var = imp.load_source("var","../../../../home/pi/aerowake-mit/Controller/var.py")
Controller = imp.load_source("Controller","../../../../../../../../home/pi/aerowake-mit/Controller/Controller.py")
Setup = imp.load_source("Setup","../../../../home/pi/aerowake-mit/Controller/Setup.py")
LogData = imp.load_source("LogData","../../../../../../../../home/pi/aerowake-mit/Controller/LogData.py")
##############################################################
# Setup
##############################################################

#Preflight Checkout--
runtime = .5  #min
run_goal = [0,20]
var.log_file_name = 'testing_log_01.csv'
standalone = True


time.sleep(2)


print "\n \n \n \n \n \n -- AeroWake Controller \n \n \n"
print " -- Running Hardware Setup Routine"

## Connection to API
#api = local_connect()
#vehAPI=api.get_vehicles()[0]

api,gcs = local_connect()
vehAPI = api.get_vehicles()[0]


#initialize Controller
print " -- Initialize Controller"
vehControl = Controller.Controller()
time.sleep(.1)

vehAPI.parameters.SR1_EXTRA1 = 10
vehAPI.parameters.SR1_POSITION = 10
vehAPI.parameters.SR1_EXTRA2 = 10

t1_0 =datetime.datetime.now()
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
    vehControl.h_mem = vehControl.uav_heading
    vehControl.uav_heading = (vehAPI.attitude.yaw*180/np.pi)  #check this
    print vehControl.relative_angle

    #what happens if you connect the GCS mid flight??
       
def update_ship():
    #print " -- Controller Ship State Updated"
    vehControl.ship_alt = 0 	 # gcs.location.alt  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)
    vehControl.ship_coord = [ 42.3555669, -71.1015294 ]# [gcs.location.lat,gcs.location.lon]  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)
    vehControl.ship_heading = 65 # vehControl.wrap360(gcs.attitude.yaw*180/np.pi) # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)
    vehControl.ship_tether_length = 9.5 	             # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)

def update_goal(g_angle):
    print " -- Controller Goal State Updated"
    vehControl.goal_angle = g_angle

#Arm Vehicle
def arm_UAV():
    print "\n  -- Arming UAV \n"
    vehAPI.channel_override = { "3" : 1000}
    vehAPI.flush()
    vehAPI.armed = True
    vehAPI.flush()
    arm_c = 0
    while not vehAPI.armed and not api.exit:
        print " -- Waiting for UAV to arm..."
        time.sleep(.5)
        arm_c+=1
        if arm_c >10:
            vehAPI.exit  #This doesnt exit the API, it throws and error, and allows the api to exit. 

    vehControl.uav_armed = vehAPI.armed
    update_goal([0,0],0,[0,5])
    print " -- Controller Status: Vehicle Armed: %s" % vehAPI.armed

def disarm_UAV():
    print "\n  -- Disarming UAV \n"
    update_goal([0,0],0,[0,0])
    #vehAPI.channel_override = {"3" : 999 }
    vehAPI.flush()
    
    if not vehControl.uav_flying:
        vehAPI.armed = False
        vehAPI.flush()

        while vehAPI.armed and not api.exit:
            print " -- Waiting for UAV to disarm..."    
            time.sleep(.5)

    print " -- Controller Status: Vehicle Armed: %s" % vehAPI.armed
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

def control_check(): #not sure what this does. Probably delete. 
    if vehControl.uav_mode != 'STABILIZE':
        human_rc_control()
    if vehControl.uav_mode == 'STABILIZE' and not vehControl.rc_write_priv:
        human_rc_control()  #controller_rc_control()   

def pilot_mode_check():
    if vehControl.uav_mode != "STABILIZE":
        return False
    else:
        return gcs.autopilot_mode
        
        
def write_channels(ch_out): 
    if vehControl.rc_write_priv and vehControl.uav_mode == 'STABILIZE':
        vehAPI.channel_override = {"1":ch_out[0], "2":ch_out[1], "3":ch_out[2], "4":ch_out[3]}
        vehAPI.flush()
        #print "---- Controller Wrote to Channels"
    else:   
        #print " ---- "        
        print "---- RC Control Active"     

##############################################################
# Mavlink Callback
##############################################################

def ship_callback():
    #usage: This will update the ship state and the goal state when it recieves a new packet from the GCS.  

    update_ship()
    
    #update_goal()
    #print " -- Ship Callback"

#Setup Attitude Callback
def uav_callback(attitude):
    #usage: This will update the UAV state in the controller whenever it receives a new packet from the telemetry link over ttyAMA0 
    update_uav()   
    ship_callback() # WRONG PLACE FOR THIS 
    control_check()
    channel_out = vehControl.run_controller() 
    write_channels(channel_out)
    
   
#vehAPI.add_attribute_observer('attitude' , uav_callback)
vehAPI.add_attribute_observer('location', uav_callback)


##############################################################
# Main Loop
##############################################################


human_rc_control()

#event: Takeoff Command Given
#event: Wait at specified altitude until a command is given
#event: Command packet recieved, execute sweep
#event: land command given

#arm_UAV()
update_goal(run_goal) # <<<<<<<<<<<<<< TEMPORARY WAY TO SET THE GOAL LOCATION -- SET ABOVE

# Run this for a bit as a test

if standalone==False:
    while True:
        control_check()
        count=0
        while pilot_mode_check():
            if count == 0:
                control_check()
            tw1= datetime.datetime.now()
        
            out_data = vehControl.compile_telem()
            LogData.write_to_log(out_data)
            if (int(count)%10)==0:
                t_rem = (10*60*runtime - count)*10
                print "---- time remaining = %.2f seconds" %(t_rem) 
            count+=1
            time.sleep(.1)
            tw2= datetime.datetime.now()
        time.sleep(0.1)
        
        
        #print "Waiting Loop"

if standalone == True:
    while True:
        control_check()
        count=0
    
        tw1= datetime.datetime.now()
        
        out_data = vehControl.compile_telem()
        LogData.write_to_log(out_data)
        
        if (int(count)%20)==0:
            t_rem = (10*60*runtime - count)*10
            print "---- time remaining = %.2f seconds" %(t_rem) 
        count+=1
        time.sleep(.05)
        tw2= datetime.datetime.now()


print "Called attitude callback ",  vehControl.control_count , " times in ", (datetime.datetime.now()-t1_0).total_seconds() ," s. Rate: ",  vehControl.control_count / (datetime.datetime.now()-t1_0).total_seconds()
print "While loop takes",  (tw2-tw1).total_seconds()

human_rc_control()
#disarm_UAV()






