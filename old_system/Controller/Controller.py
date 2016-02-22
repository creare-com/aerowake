#Name: Controller.py
#Author: mklinker
#Usage: Flight scheduler and Controller (PID's and such)

import numpy as np
import imp
import datetime

#var = imp.load_source("var","../../../../home/pi/aerowake-mit/Controller/var.py")
import var

class Controller:
    def __init__(self):
        #self.description = "flight scheduler and controller to handle high level commands"
        self.uav_coord = [1,1]      #GPS coordinates of UAV [lat,lon] [degrees]
        self.uav_vel = [0,0,0]      #UAV velocity [m/s] [x,y,z]    
        self.uav_alt = 0            #UAV Altitude [m]
        self.uav_mode = 0           #UAV Mode
        self.uav_attitude = [0,0]   #Current UAV attitude (roll,pitch)[radians]
        self.uav_heading = 0        #UAV heading (+- 180 degrees). [Deg]. 
        self.uav_airspeed = 0       #aispeed in [m/s] via the pitot tube
        self.uav_vel_rel = [0,0]

    #todo: make one function that updates all of these fields
        self.goal_attitude = [0,0]  #Goal UAV attitude (roll,pitch)[radians]    
        self.goal_angle = [0,0]     #Goal UAV Angles (theta, phi) [degrees]
    
        self.ship_alt = 0           #GPS altitude of ship station (tether) [m]
        self.ship_vel = [0,0,0]   #GPS Velocity of ship station [m/s] [x,y,z] Should be all X. 
        self.ship_coord = [0,0]     #GPS position of ship station (tether) [lat,lon] [degrees]
        self.ship_heading = 0       #Heading of ship [degrees] +- 180 degrees
        self.ship_tether_length = 0             #Tether length [m]
        self.ship_airspeed = 0  # ship airspeed in [m/s] @ pitot tube. 


        self.uav_flying = False     #Flag for if the UAV is flying or not
        self.uav_failsafe = False   #Flag for if the UAV is in failsafe or not
        self.uav_armed = False      #Flag for if the UAV is armed or not. 
        self.rc_write_priv = False  #Flag for whether the controller can write to RC Override or not. 
        
    #todo: make one function that updates all of these fields
        self.relative_position = [0,0,0] #UAV position relative to ship. (x,y,z) [m]. X is parallel with ship(forward positive), Y perpendicular to ship, Z is alt. 
        self.relative_angle = [0,0] #UAV angles off stern of ship. (theta,phi) [degrees] theta=horizontal, phi=vertical
        self.pose_hold_effort = [0,0,0] #Forces the UAV is trying to exert to stay put. XYZ 
        self.xyz_ctrl_effort = [0,0,0] #Forces from the controller in XYZ plane
        self.control_output = [0,0,0,0] #Forces the UAV is trying to exert in the spherical        
        self.spherical_vel = [0,0,0]

        self.control_count = 0
        self.h_mem = 0
        self.t_mem = datetime.datetime.now()
        self.prev_e = np.array([0,0])
        self.e_int = np.array([0,0])
        
###############################################################################################
### Utility Functions
###############################################################################################

    def wrap360(self,val):
        if val>360:
            val=val-360
        if val<0:
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
        act1 = self.ship_coord #SHIP FIRST
        act2 = self.uav_coord #UAV SECOND
        #dlat=act1[0]-act2[0]
        dlon=act1[1]-act2[1] #ship - uav
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
        
    def pitch_2_pwm(self,angle):
        angle=angle
        return int( self.saturate(1500+(angle*500/45)  ,1000,2000))

    def roll_2_pwm(self,angle):
        angle=angle
        return int(self.saturate(1500-(angle*500/45),1000,2000))
    
    def yaw_2_pwm(self,yaw_input):
        return int(self.saturate(1500+yaw_input,1400,1600))

    def set_spherical_velocities(self): # <<<<<<<<<<<<<< Needs to be tested.  Not sure about directions. 
        # Set up velocity mappings
        # velocity_x > 0 => fly North
        # velocity_y > 0 => fly East
        # velocity_z < 0 => ascend
    
        v_uav_rel_global = np.array(self.uav_vel) - np.array(self.ship_vel) #This is north south up, not relative to ship. 
        vN = v_uav_rel_global[0] 
        vE = v_uav_rel_global[1] 
        vz = v_uav_rel_global[2]
        
        vx = -(vN*np.cos(self.ship_heading*np.pi/180) + vE*np.sin(self.ship_heading*np.pi/180)) # <<<<<<<<<<<<<<<<<<<<< Should this be ship or UAV?#
        vy =  -vN*np.sin(self.ship_heading*np.pi/180) + vE*np.cos(self.ship_heading*np.pi/180) 
        #print('vel X %.4f   vel Y %.4f \n') %(vx,vy)
 
        self.uav_vel_rel = [vx,vy]
               
        th =  self.relative_angle[0]*np.pi/180 
        phi = self.relative_angle[1]*np.pi/180
    
        th_dot = vy*np.cos(th) - vx*np.sin(th) 
        phi_dot = vz*np.cos(phi) - vx*np.sin(phi)*np.cos(th) - vy*np.sin(phi)*np.sin(th)
        r_dot = vz*np.sin(phi) + vx*np.cos(phi)*np.cos(th) + vy*np.sin(th)*np.cos(phi)
        
        self.spherical_vel = [th_dot, phi_dot ,r_dot]

        
###############################################################################################
### UAV Operation Functions
###############################################################################################

    def run_controller(self):
        #make sure everything is updated here.      
        # F_tension, R, relative angles, drag forces, velocities
        self.control_count +=1   
        
        dt = (datetime.datetime.now() - self.t_mem).total_seconds()
        self.t_mem = datetime.datetime.now()

        self.set_relative_angle() 
        self.set_spherical_velocities()        
        
        drag_forces = self.get_drag_forces()
        #sph_vel = self.spherical_vel  # <<<<<<<<<<<<<<<<<<<<<<<< Untested. 
        #angle_vel = [sph_vel[0],sph_vel[1]]         

        th =  self.relative_angle[0]*np.pi/180 
        phi = self.relative_angle[1]*np.pi/180

        ft= 5 #newtons  #  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< need a way to measure or estimate
        
        #Calculate force needed to maintain position on tether
        fx = (ft)*np.cos(th)*np.cos(phi) - drag_forces[0]
        fy = (ft)*np.sin(th)*np.cos(phi) - drag_forces[1]
        fz = (ft)*np.sin(phi) + var.m*var.g
       
        #Calculate azmuth/inclination angle error
        e_angle = np.array(self.goal_angle)*np.pi/180 - np.array([th,phi])  #[theta, phi]
        try:
            e_angle_dot = (e_angle - self.prev_e)/dt
        except:
            print '---- dt=0'
            e_angle_dot = 0            
        self.prev_e = e_angle
        self.e_int += e_angle*dt       
        
        f_in = var.kp_pose*e_angle + var.kd_pose*e_angle_dot + var.ki_pose*self.e_int

        #Calculate forces to move the UAV given the control inputs.         
        fix = -f_in[0]*np.sin(th) - f_in[1]*np.sin(phi)
        fiy =  f_in[0]*np.cos(th)
        fiz =  f_in[1]*np.cos(phi)
        
        #Saturate the inputs. 
        fin_t = np.sqrt(fix**2 + fiy**2 + fiz**2)
        
        thres = 5
        if abs(fin_t)>thres:
            fix = thres/abs(fin_t) * fix
            fiy = thres/abs(fin_t) * fiy
            fiz = thres/abs(fin_t) * fiz
            #print ' ---- Controls Saturated '+str(int(fin_t))
        
        #Add the forces
        ftx = float(fx + fix)
        fty = float(fy + fiy)
        ftz = float(fz + fiz)+0.000000001 #<<<<<<<<<<<<<<<<<<<< Was getting NaN due to division below. 
        
        #Calculate Roll, Pitch, Throttle
        f_total = np.sqrt(ftx**2 + fty**2 + ftz**2)

        pitch_out = (np.arctan(ftx/ftz))*180/np.pi  
        roll_out =  (np.arctan(fty/ftz))*180/np.pi 
        thr_out = (f_total)
  
        #Calculate yaw errors OR command yaw to be aligned with ship
        e_heading = self.ship_heading - self.uav_heading 
        
        try:
            yaw_vel = (self.uav_heading - self.h_mem)/dt
        except:
            print '---- dt = 0'
            yaw_vel = 0
        
        #PD controller on yaw
        yaw_in = var.kp_yaw*e_heading - var.kd_yaw*yaw_vel     
        
        self.goal_attitude=[pitch_out,roll_out,thr_out] #
        self.pose_hold_effort = [fx,fy,fz] #Forces the UAV is trying to exert to stay put. XYZ 
        self.xyz_ctrl_effort = [fix,fiy,fiz] #Forces from the controller in XYZ plane
        self.control_output = [self.pitch_2_pwm(pitch_out),self.roll_2_pwm(roll_out),self.force_2_thr_pwm(thr_out),self.yaw_2_pwm(yaw_in)] #Forces the UAV is trying to exert in the spherical	

        return self.control_output #[ch1,ch2,ch3,ch4]
              
              
              
              

              
              
    def compile_telem(self):
        roll = float(self.uav_attitude[0])
        pitch = float(self.uav_attitude[1]) 
        yaw = float(self.uav_heading)
        throttle = 1.23
        
        vox = float(self.uav_vel[0])
        voy = float(self.uav_vel[1])
        voz = float(self.uav_vel[2]) 
        vth = float(self.spherical_vel[0])
        vphi = float(self.spherical_vel[1])
        vr = float(self.spherical_vel[2])
        
        v_rel_x = float(self.uav_vel_rel[0])
        v_rel_y = float(self.uav_vel_rel[1])
        
        airspd = float(self.uav_airspeed)
        lat = float(self.uav_coord[0])
        lon = float(self.uav_coord[1])
        alt = float(self.uav_alt)    
        
        s_lat = float(self.ship_coord[0])
        s_lon = float(self.ship_coord[1])
        s_alt = float(self.ship_alt)
        s_head= float(self.ship_heading)
        s_vel_x = float(self.ship_vel[0])
        s_vel_y = float(self.ship_vel[1])
        s_vel_z = float(self.ship_vel[2])        
        t_dist = float(self.get_diagonal_distance()) 
        
        th = float(self.relative_angle[0])
        phi = float(self.relative_angle[1])
        
        goal_th = float(self.goal_angle[0])
        goal_phi = float(self.goal_angle[1])
        
        fx = float(self.pose_hold_effort[0])
        fy = float(self.pose_hold_effort[1])    
        fz = float(self.pose_hold_effort[2])
        
        fix= float(self.xyz_ctrl_effort[0])
        fiy= float(self.xyz_ctrl_effort[1])
        fiz= float(self.xyz_ctrl_effort[2])
        
        p = float(self.control_output[0])
        r = float(self.control_output[1])
        thr = float(self.control_output[2])
        yw =  float(self.control_output[3])
    
        #need 31
        out = "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f" %(roll,pitch,yaw,throttle,vox,voy,voz,vth,vphi,vr,airspd,lat,lon,alt,th,phi,goal_th,goal_phi,fx,fy,fz,fix,fiy,fiz,p,r,thr,yw,s_lat,s_lon,t_dist,s_alt,s_head,s_vel_x,s_vel_y,s_vel_z,v_rel_x,v_rel_y)
        return out

#        out = "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f" %(roll,pitch,yaw,throttle,vox,voy,voz,vth,vphi,vr,airspd,lat,lon,alt,s_lat,s_lon,t_dist,th,phi,goal_th,goal_phi,fx,fy,fz,fix,fiy,fiz,p,r,thr,yw)



