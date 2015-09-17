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
        self.uav_vel = [0,0,0]      #UAV velocity [m/s] [x,y,z]    
        self.uav_alt = 0            #UAV Altitude [m]
        self.uav_mode = 0           #UAV Mode
        self.uav_attitude = [0,0]   #Current UAV attitude (roll,pitch)[radians]
        self.uav_heading = 0        #UAV heading (0 to 360). [Deg]. 
        self.uav_airspeed = 0       #aispeed in [m/s] via the pitot tube


    #todo: make one function that updates all of these fields
        self.goal_attitude = [0,0]  #Goal UAV attitude (roll,pitch)[radians]    
        self.goal_alt = 0           #Goal UAV Altitude [m]
        self.goal_angle = [0,0]     #Goal UAV Angles (theta, phi) [degrees]
    
        self.ship_alt = 0           #GPS altitude of ship station (tether) [m]
        self.ship_vel = [0,0,0]   #GPS Velocity of ship station [m/s] [x,y,z] Should be all X. 
        self.ship_coord = [0,0]     #GPS position of ship station (tether) [lat,lon] [degrees]
        self.ship_heading = 0       #Heading of ship [degrees]
        self.ship_tether_length = 0             #Tether length [m]
        self.ship_airspeed = 0  # ship airspeed in [m/s] @ pitot tube. 


        self.uav_flying = False     #Flag for if the UAV is flying or not
        self.uav_failsafe = False   #Flag for if the UAV is in failsafe or not
        self.uav_armed = False      #Flag for if the UAV is armed or not. 
        self.rc_write_priv = False #Flag for whether the controller can write to RC Override or not. 
        
    #todo: make one function that updates all of these fields
        self.relative_position = [0,0,0] #UAV position relative to ship. (x,y,z) [m]. X is parallel with ship(forward positive), Y perpendicular to ship, Z is alt. 
        self.relative_angle = [0,0] #UAV angles off stern of ship. (theta,phi) [degrees] theta=horizontal, phi=vertical
        self.pose_hold_effort = [0,0,0] #Forces the UAV is trying to exert to stay put. XYZ 
        self.xyz_ctrl_effort = [0,0,0] #Forces from the controller in XYZ plane
        self.sph_ctrl_effort = [0,0,0] #Forces the UAV is trying to exert in the spherical        
        self.spherical_vel = [0,0,0]

        self.control_count = 0
        
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
        #act1 =  [42.3578720 ,-71.0979608 ]
        #act2 = [ 42.3579384 , -71.0977609 ]
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
        act1 = self.ship_coord
        act2 = self.uav_coord
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
        theta = (180 - (g_ang - self.ship_heading))
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

    def get_drag_forces(self):
       return [0,0] #  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
    def angle_2_pwm(self,angle):
        angle=angle
        return int( self.saturate(1500+angle*(2000/np.pi)  ,1000,2000))

#    def get_spherical_velocities(self):  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Needs to be tested.  Not sure about directions. 
#        
#        v_uav_rel_global = np.array(self.uav_vel) - np.array(self.ship_vel) #This is north south up, not relative to ship. 
#        vE = v_uav_rel[0] 
#        vN = v_uav_rel[1] 
#        vz = v_uav_rel[2]
#        
#        vx = -(vN*np.cos(self.ship_heading*np.pi/180) + vE*np.sin(self.ship_heading*np.pi/180)) # <<<<<<<<<<<<<<<<<<<<< Should this be ship or UAV?
#        vy =  -vN*np.sin(self.ship_heading*np.pi/180) + vE*np.cos(self.ship_heading*np.pi/180) 
#        
#        th =  self.relative_angle[0]*np.pi/180 
#        phi = self.relative_angle[1]*np.pi/180
#
#        th_dot = vy*np.cos(th) - vx*np.sin(th) 
#        phi_dot = vz*np.cos(phi) - vx*np.sin(phi)*np.cos(th) - vy*np.sin(phi)*np.sin(th)
#        r_dot = vz*np.sin(phi) + vx*np.cos(phi*np.cos(th) + vy*np.sin(th)*np.cos(phi)
#        
#        return 5#[th_dot, phi_dot ,r_dot]

    def set_spherical_velocities(self): # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Needs to be tested.  Not sure about directions. 
        v_uav_rel_global = np.array(self.uav_vel) - np.array(self.ship_vel) #This is north south up, not relative to ship. 
#        vE = v_uav_rel[0] 
#        vN = v_uav_rel[1] 
#        vz = v_uav_rel[2]
        
#        vx = -(vN*np.cos(self.ship_heading*np.pi/180) + vE*np.sin(self.ship_heading*np.pi/180)) # <<<<<<<<<<<<<<<<<<<<< Should this be ship or UAV?#
#        vy =  -vN*np.sin(self.ship_heading*np.pi/180) + vE*np.cos(self.ship_heading*np.pi/180) 

#        th =  self.relative_angle[0]*np.pi/180 
#        phi = self.relative_angle[1]*np.pi/180
#
#        th_dot = vy*np.cos(th) - vx*np.sin(th) 
#        phi_dot = vz*np.cos(phi) - vx*np.sin(phi)*np.cos(th) - vy*np.sin(phi)*np.sin(th)
#        r_dot = vz*np.sin(phi) + vx*np.cos(phi*np.cos(th) + vy*np.sin(th)*np.cos(phi)
        
#        self.spherical_vel = [th_dot, phi_dot ,r_dot]
        self.spherical_vel = [5,5,5]
        
        
        
        
###############################################################################################
### UAV Operation Functions
###############################################################################################

    def run_controller(self):
        #make sure everything is updated here.      
        # F_tension, R, relative angles, drag forces, velocities
        self.control_count +=1        

        self.set_relative_angle() 
        self.set_spherical_velocities()        
        
        drag_forces = self.get_drag_forces()
        sph_vel = self.spherical_vel  # <<<<<<<<<<<<<<<<<<<<<<<< Untested. 
        angle_vel = [sph_vel[0],sph_vel[1]]         

        th =  self.relative_angle[0]*np.pi/180 
        phi = self.relative_angle[1]*np.pi/180

        ft= 5 #newtons  #  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< need a way to measure or estimate
        
        #Calculate force needed to maintain position on tether
        fx = (ft)*np.cos(th)*np.cos(phi) - drag_forces[0]
        fy = (ft)*np.sin(th)*np.cos(phi) - drag_forces[1]
        fz = (ft)*np.sin(phi) + var.m*var.g
       
        #Calculate azmuth/inclination angle error
        e_angle = np.array(self.goal_angle)*np.pi/180 - np.array([th,phi])  #[theta, phi]
        f_in = var.kp_pose*e_angle - var.kd_pose*angle_vel

        #print f_in
        
        #Calculate forces to move the UAV given the control inputs.         
        fix = -f_in[0]*np.sin(th) - f_in[1]*np.sin(phi)
        fiy = f_in[0]*np.cos(th)
        fiz = f_in[1]*np.cos(phi)
        
        #Add the forces
        ftx = float(fx + fix)+.000000001 #<<<<<<<<<<<<<<<<<<<< Was getting NaN due to division below. 
        fty = float(fy + fiy)+.000000001
        ftz = float(fz + fiz)+.000000001
        
        #Calculate Roll, Pitch, Throttle
        f_total = np.sqrt(ftx**2 + fty**2 + ftz**2)

        pitch_out = self.angle_2_pwm( np.arctan(ftz/ftx))  #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< make sure this is right!!!!!!!!!!!!! 
        roll_out = self.angle_2_pwm( np.arctan(ftz/fty)) 
        thr_out = self.force_2_thr_pwm(f_total)
        
                # Store some data  
        self.pose_hold_effort = [fx,fy,fz] #Forces the UAV is trying to exert to stay put. XYZ 
        self.xyz_ctrl_effort = [fix,fiy,fiz] #Forces from the controller in XYZ plane
        self.sph_ctrl_effort = [pitch_out,roll_out,thr_out] #Forces the UAV is trying to exert in the spherical 
        
        self.goal_attitude=[1,1] # <<<<<<<<< Fill in
  
        #Calculate yaw errors OR command yaw to be aligned with ship
        e_heading = self.ship_heading - self.uav_heading 
        yaw_in = var.kp_yaw*e_heading 
        #hand back [rollPWM, pitchPWM, throttlePWM, yawPWM]     

        return [roll_out,pitch_out,thr_out,1500] #[ch1,ch2,ch3,ch4]



