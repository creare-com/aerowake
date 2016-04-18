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
import gps
import threading

var = imp.load_source("var","../../../../home/pi/aerowake-mit/Controller/var.py")
Controller = imp.load_source("Controller","../../../../../../../../home/pi/aerowake-mit/Controller/Controller.py")
Setup = imp.load_source("Setup","../../../../home/pi/aerowake-mit/Controller/Setup.py")
LogData = imp.load_source("LogData","../../../../../../../../home/pi/aerowake-mit/Controller/LogData.py")
##############################################################
# Setup
##############################################################

#Preflight Checkout--
runtime =  .1 #min
run_goal = [0,20]
var.log_file_name = 'GPS_testing_0005.csv'
standalone=True

time.sleep(2)



print "\n \n \n \n \n \n -- AeroWake Controller \n \n \n"

## Connection to API
#api = local_connect()
#vehAPI=api.get_vehicles()[0]

api,gcs = local_connect()
vehAPI = api.get_vehicles()[0]


#initialize Controller
print " -- Initialize Controller"
vehControl = Controller.Controller()
time.sleep(.1)

# GPS Service
gpsd_ses = gps.gps("localhost","2947")
gpsd_ses.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
report =gpsd_ses.next()
print ' -- GPSD Setup Complete'

time.sleep(1)
t1_0 =datetime.datetime.now()

##############################################################
# State Update Functions
##############################################################


#Update UAV Controller State
def update_uav():
    #print " -- Controller UAV State Updated"
    vehControl.uav_alt = vehAPI.location.alt
    #vehControl.uav_coord = [vehAPI.location.lat,vehAPI.location.lon]
    vehControl.uav_mode = vehAPI.mode.name
    vehControl.uav_attitude = [ vehAPI.attitude.roll , vehAPI.attitude.pitch ]
    vehControl.uav_vel = vehAPI.velocity
    vehControl.h_mem = vehControl.uav_heading
    vehControl.uav_heading = (vehAPI.attitude.yaw*180/np.pi)  #check this
    #print '----- '
    #print '%.2f     %.2f     %.2f ' %(vehControl.relative_angle[0],vehControl.relative_angle[1],vehControl.get_distance())
    #print '----- ' 
       
def update_ship():
    #print " -- Controller Ship State Updated"
    vehControl.ship_alt = 0                                                             #gcs.gcs_alt  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)
    vehControl.ship_coord = [42.35854040,-71.09724260]      #[gcs.gcs_lat,gcs.gcs_lon]  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)
    vehControl.ship_heading = 60                                                #gcs.gcs_yaw*180/np.pi #vehControl.wrap360(gcs.attitude.yaw*180/np.pi) # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)
    vehControl.ship_tether_length = 10 	                                      # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)

def update_goal(g_angle):
    print " -- Controller Goal State Updated"
    vehControl.goal_angle = g_angle


## GPS THREAD
def gps_thread():
    print 'i got called'
    while True:
        print 'i got looped'
        new_pose = gps_update()
        if new_pose!=0:
            vehControl.uav_coord = new_pose
            print vehControl.uav_coord
        time.sleep(5/100.0)

t_gps = threading.Thread(name='gps_thread', target = gps_thread)

def gps_update():
    print ' -- GPS Loop Triggered -- '
    try:
        report =gpsd_ses.next()
        if hasattr(report,'time'):
            print report.time
        else:
            print ' -- Nothing in GPSD Report -- '
            return 0

        if report['class'] == 'TPV':
            if hasattr(report,'lat'):
                print ' -- Got GPSD Update --   lat: ',report.lat,'  lon: ',report.lon
                return [report.lat,report.lon]
        else:
            print " -- No GPSD Update -- "
            return 0
    except StopIteration:
        #gpsd_ses = None
        print "GPSD HAS TERMINATED"
        return 0
        

#############################################################
# Control Checks and Control Outputting
#############################################################

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

##############################################################
# Action Functions 
##############################################################

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
    print " -- Controller Status: Vehicle Armed: %s" % vehAPI.armed

def disarm_UAV():
    print "\n  -- Disarming UAV \n"
    vehAPI.armed = False
    vehAPI.flush()

        while vehAPI.armed and not api.exit:
            print " -- Waiting for UAV to disarm..."    
            time.sleep(.5)

    print " -- Controller Status: Vehicle Armed: %s" % vehAPI.armed
    vehControl.uav_armed = vehAPI.armed


##############################################################
# Mavlink Callback
##############################################################

def ship_callback():
    #usage: This will update the ship state and the goal state when it recieves a new packet from the GCS.  
    update_ship()


#Setup Attitude Callback
def uav_callback(attitude):
    #usage: This will update the UAV state in the controller whenever it receives a new packet from the telemetry link over ttyAMA0 
    update_uav()   
    ship_callback() # WRONG PLACE FOR THIS 
    control_check()
    channel_out = vehControl.run_controller() 
    write_channels(channel_out)
    
   
vehAPI.add_attribute_observer('attitude' , uav_callback)
#vehAPI.add_attribute_observer('location', uav_callback)


##############################################################
# Main Loop
##############################################################

t_gps.start()

human_rc_control()

update_goal(run_goal) # <<<<<<<<<<<<<< TEMPORARY WAY TO SET THE GOAL LOCATION -- SET ABOVE

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
            if (int(count)%30)==0:
                t_rem = (10*60*runtime - count)*10
                print "---- time remaining = %.2f seconds" %(t_rem) 
            count+=1
            time.sleep(.1)
            tw2= datetime.datetime.now()
        time.sleep(0.1)
    print "Waiting Loop"




arm_UAV()
vehAPI.channel_override = { "1":1500, "2":1500, "3":1500, "4":1500, "5":0, "6":0, "7":0, "8":0}

count=0
if standalone==True:
    while True:
        print '----------------------------------------------------------------- '
        control_check()      

        tw1=datetime.datetime.now()
        out_data = vehControl.compile_telem()
        LogData.write_to_log(out_data)
        count+=1
        time.sleep(.1)
        tw2= datetime.datetime.now()
        print count
        if count>runtime*60*10:
            break
        
vehAPI.channel_override = { "1":0, "2":0, "3":0, "4":0, "5":0, "6":0, "7":0, "8":0}

gpsd_ses.running = False

print "Called attitude callback ",  vehControl.control_count , " times in ", (datetime.datetime.now()-t1_0).total_seconds() ," s. Rate: ",  vehControl.control_count / (datetime.datetime.now()-t1_0).total_seconds()
print "While loop takes",  (tw2-tw1).total_seconds()

human_rc_control()
disarm_UAV()

print ' \n\n\n\n\n\n\n\n AeroWake Controller Exiting \n\n'

t_gps.terminate()







