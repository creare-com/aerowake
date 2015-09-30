#import time
#import datetime
#import smbus
#import numpy as np
#from droneapi.lib import VehicleMode
#from pymavlink import mavutil
#import signal, sys
import time 


import numpy as np
import Controller
import LogData
print '\n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n '
vehControl = Controller.Controller()

def update_uav():
    #print " -- Controller UAV State Updated"
    vehControl.uav_alt = 0
    vehControl.uav_coord = [ 42.3561080, -71.1008495 ] #3
    vehControl.uav_attitude = [0,0]
    vehControl.uav_vel = [1,-1,0] #North, East, Up
    vehControl.h_mem = 48
    vehControl.uav_heading = 0#vehControl.wrap360(-20)  #check this
    #what happens if you connect the GCS mid flight??
       
def update_ship():
    #print " -- Controller Ship State Updated"
    vehControl.ship_alt = 0 	 # gcs.location.alt  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)
   
    #vehControl.ship_coord = [42.3558741,-71.1015308] #4    
    vehControl.ship_coord = [42.3556145,-71.1008361] #5
    #vehControl.ship_coord = [42.3558741,-71.1015308] #6
    vehControl.ship_heading = 45 # vehControl.wrap360(gcs.attitude.yaw*180/np.pi) # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)
    vehControl.ship_tether_length = 60 	             # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HardCoded (remove)

def update_goal():
    vehControl.goal_angle = [0,30] 

update_uav()
update_ship()
update_goal()
time.sleep(.1)
vehControl.run_controller()


print ' '
print 'global angle   = '+str(vehControl.get_global_angle())
print 'relative angle = ' + str(vehControl.relative_angle)
print 'distance       = '+str(vehControl.get_diagonal_distance())
print ' '
print 'pose hold      = ' + str(vehControl.pose_hold_effort)
print ' '
print 'xyz cntrl efft = ' + str(vehControl.xyz_ctrl_effort)
print ' '
print 'pitch roll     = ' + str(vehControl.goal_attitude)
print ' '
print 'pwms           = ' + str(vehControl.control_output)
print ' '
print 'Theta_vel       = ' + str(vehControl.spherical_vel[0])
print 'Phi_vel         = ' + str(vehControl.spherical_vel[1])
print 'R_vel           = ' + str(vehControl.spherical_vel[2])
print ' '

out_data = vehControl.compile_telem()
#LogData.write_to_log(out_data)














