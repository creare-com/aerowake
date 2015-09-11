#Name: Controller.py
#Author: mklinker
#Usage: Flight scheduler and Controller (PID's and such)

import numpy as np
import imp

var = imp.load_source("var","../../../../home/pi/aerowake-mit/Controller/var.py")

class Controller:
    def __init__(self):
        self.description = "flight scheduler and controller to handle high level commands"
        self.uav_coord = [1,1]      #GPS coordinates of UAV [lat,lon] [degrees]
        self.uav_vel = [0,0,0]      #UAV velocity [m/s]     
        self.uav_alt = 0            #UAV Altitude [m]
        self.uav_mode = 0           #UAV Mode
        self.uav_attitude = [0,0]       #Current UAV attitude (roll,pitch)[radians]
        self.uav_heading = 0

    #todo: make one function that updates all of these fields
        self.goal_attitude = [0,0]  #Goal UAV attitude (roll,pitch)[radians]    
        self.goal_alt = 0           #Goal UAV Altitude [m]
        self.goal_angle = [0,0]     #Goal UAV Angles (theta, phi) [degrees]
    
        self.ship_alt = 0           #GPS altitude of ship station (tether) [m]
        self.ship_coord = [0,0]     #GPS position of ship station (tether) [lat,lon] [degrees]
        self.ship_heading = 0       #Heading of ship [degrees]
        self.ship_tether_length = 0             #Tether length [m]

        self.uav_flying = False     #Flag for if the UAV is flying or not
        self.uav_failsafe = False   #Flag for if the UAV is in failsafe or not
        self.uav_armed = False      #Flag for if the UAV is armed or not. 
        self.rc_write_priv = False #Flag for whether the controller can write to RC Override or not. 
        
    #todo: make one function that updates all of these fields
        self.relative_position = [0,0,0] #UAV position relative to ship. (x,y,z) [m]. X is parallel with ship(forward positive), Y perpendicular to ship, Z is alt. 
        self.relative_angle = [0,0] #UAV angles off stern of ship. (theta,phi) [degrees] theta=horizontal, phi=vertical
        
###############################################################################################
### Utility Functions
###############################################################################################

    def wrap360(self,val):
        if val>360:
            val=val-360
        if val<360:
            val=val+360
        return val

    def saturate(self,val,lower,upper):
        if val>upper:
            val=upper
        if val<lower:
            val=lower
        return val
###############################################################################################
### Geometric Functions
###############################################################################################

    def get_distance(self): #Tested separately. Gives the 2D GPS position distance from the ship to UAV
        act1 = self.uav_coord
        act2 = self.ship_coord
        R = 6371229
        dlat = act1[0]-act2[0]
        dlon = act1[1]-act2[1]
        a = (np.sin(dlat/2*np.pi/180))**2 + np.cos(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * (np.sin(dlon/2*np.pi/180))**2
        distance = R *2 * np.arctan2(np.sqrt(a),np.sqrt(1-a))
        return distance #[meters]

    def get_diagonal_distance(self): #Gives the ideal taut-tether length from the ship to the UAV. 
        xy_dist = self.get_distance()
        z_dist = self.uav_alt-self.ship_alt
        diag_dist = ( xy_dist**2 + z_dist**2 )**0.5
        return diag_dist #[meters]

    def get_global_angle(self): #Tested separately. Gives the heading angle from North from the ship to UAV. 
        act1 = self.uav_coord
        act2 = self.ship_coord
        dlat=act1[0]-act2[0]
        dlon=act1[1]-act2[1]
        arg1= np.sin(dlon*np.pi/180) * np.cos(act2[0]*np.pi/180) 
        arg2= np.cos(act1[0]*np.pi/180) * np.sin(act2[0]*np.pi/180) - np.sin(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * np.cos(dlon*np.pi/180)        
        g_rel_ang =  -np.arctan2( arg1, arg2)
        if g_rel_ang<0:
            g_rel_ang=g_rel_ang+2*np.pi
        g_rel_ang=g_rel_ang*180/np.pi
        return g_rel_ang #[degrees]

    def set_relative_angle(self): #Angle between stern of ship and the uav. 
        g_ang = self.get_global_angle()  #degrees
        theta = self.wrap360(180 - (g_ang - self.ship_heading))
        phi = np.arctan( (self.uav_alt-self.ship_alt) / self.get_distance() ) *180/np.pi
        self.relative_angle = [theta, phi]
    
    def get_relative_position(self):
        xy_dist = self.get_distance()
        z = self.uav_alt-self.ship_alt
        theta = self.relative_angle[0] 
        x = xy_dist*np.cos(theta*np.pi/180)
        y = xy_dist*np.sin(theta*np.pi/180)
        self.relative_position=[x,y,z]
        return self.relative_position
        
    def force_2_thr_pwm(self,force):
        return int(self.saturate(1000+force*10,1000,2000))
        
    def angle_2_pwm(self,angle):
        return int( self.saturate(1500+angle*(2000/np.pi)  ,1000,2000))
        
    def get_drag_forces(self):
        return [0,0] #  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    def get_spherical_velocities(self):
        th_dot = 0
        phi_dot = 0
        return [th_dot,phi_dot]    
###############################################################################################
### UAV Operation Functions
###############################################################################################


    def run_controller(self):
        #make sure everything is updated here.      
        # F_tension, R, relative angles, drag forces, velocities

        self.set_relative_angle() 
        drag_forces = self.get_drag_forces()
        angle_vel = self.get_spherical_velocities()        
        
        th = self.relative_angle[0] 
        phi = self.relative_angle[1]

        ft= 10 #newtons  #  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< need a way to measure or estimate
        
        #Calculate force needed to maintain position on tether
        fx = (ft)*np.cos(th)*np.cos(phi) - drag_forces[0]
        fy = (ft)*np.sin(th)*np.cos(phi) - drag_forces[1]
        fz = (ft)*np.sin(phi) + var.m*var.g
        
        #Calculate azmuth/inclination angle error
        e_angle = np.array(self.goal_angle) - np.array(self.relative_angle)  #[theta, phi]
        f_in = var.kp_pose*e_angle - var.kd_pose*angle_vel

        #Calculate forces to move the UAV given the control inputs.         
        fix = -f_in[0]*np.sin(th) - f_in[1]*np.sin(phi)
        fiy = f_in[0]*np.cos(th)
        fiz = f_in[1]*np.cos(phi)
        
        #Add the forces
        ftx = fx + fix
        fty = fy + fiy
        ftz = fz + fiz
        
        #Calculate Roll, Pitch, Throttle
        f_total = np.sqrt(ftx**2 + fty**2 + ftz**2)

        pitch_out = self.angle_2_pwm( 0)  #<<<<<<<<<<<<<<<<< Fill in -- look @ matlab script
        roll_out = self.angle_2_pwm( 0 ) 
        thr_out = self.force_2_thr_pwm( f_total )
        
        self.goal_attitude=[1,1] # <<<<<<<<< Fill in
  
        #Calculate yaw errors OR command yaw to be aligned with ship
        e_heading = self.ship_heading - self.uav_heading 
        yaw_in = var.kp_yaw*e_heading 
        #hand back [rollPWM, pitchPWM, throttlePWM, yawPWM]     
        return [roll_out,pitch_out,thr_out,1500] #[ch1,ch2,ch3,ch4]



