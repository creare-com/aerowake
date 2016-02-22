#!/usr/bin/env python2


import numpy as numpy
import imp
import datetime


class pose_control:
	def __init__(self):

		# State Information
		self.uav_coord = [0,0]  	# GPS Coordinates of UAV [lat,lon] from pixhawk (DD.DDDDDDD)
		self.uav_pose = [0,0,0]		# UAV Position [theta,phi,r] (radians)
		self.uav_vel = [0,0,0]		# UAV velocity [x,y,z] from pixhawk (m/s)
		self.uav_alt = 0			# UAV Alt from pixhawk (m)
		self.uav_heading = 0		# UAV Heading (degrees)
 
		self.gcs_coord = [0,0]   	# GPS Coordinates of GCS [lat,lon] from pixhawk (DD.DDDDDD)
		self.gcs_vel = [0,0,0]		# GCS Velocity [x,y,z] from pixhawk (m/s)
		self.gcs_alt = 0			# GCS Altitude from pixhawk (m)
		self.gcs_heading = 0		# GCS Heading (degrees)
		self.gcs_tether_l = 0		# GCS Tether Length (m)
        self.gcs_tether_tension = 0 # GCS Tether Tension (newtons)

		# Inputs 
		self.goal_pose = [0,0,0]	# UAV Goal Position [theta,phi,r] (radians)

		# Outputs
		self.goal_attitude = [0,0] 	# UAV Goal attitude [roll, pitch] (radians)
		self.goal_throttle = 0		# UAV Goal Throttle (PWM Signal)
		self.goal_heading = 0 		# UAV Goal Heading (degrees) 
		
		# Memory Items for Control
		self.control_c = 0


        # Controller Gains
        self.k_phi =[1.2, 2.0,  1.0] 
        self.k_th  =[1,   2.0,  1.0]
        self.k_r   =[.5,   3] 
        self.ft    = 1 #Extra tension to add to tether. (newtons)


##!!!!!!!!!!!!!!!!!!!!!!!!!! Helper Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    def saturate(self,val,lower,upper):
        if val>upper:
            val=upper
        if val<lower:
            val=lower
        return val

##!!!!!!!!!!!!!!!!!!!!!!!! Geometry Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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


##!!!!!!!!!!!!!!!!!!!!!!!!!! Mapping Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    def thr_2_force(self,force):
    	thr = force * throttle_scaling
    	return int(self.saturate(thr,0,1))

##!!!!!!!!!!!!!!!!!!!!!!!!!!!! Estimation Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	def get_drag_vector(self):
		return [0,0,0]

	def get_tether_vector(self,r_in,th,phi):
        fx = -(r_in)*np.cos(phi)*np.sin(th);
        fy = -(r_in)*np.sin(phi)*np.sin(th);
        fz = (r_in)*np.cos(th) + self.f_hover;
		return [fx,fy,fz]




##!!!!!!!!!!!!!!!!!!!!!!!!!!!! Run Controller Function !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	def run_pose_controller(self):
		self.control_c +=1

        control_dt = 

        th = self.get_theta()
        phi = self.get_phi()
        r = self.get_r()

        #### Forces to move #####

        # error_dot
        e_phi_dot = 
        e_th_dot = 
        e_r_dot = 

        # error
        self.e_phi = r*(self.goal_phi - phi)
        self.e_th = r*(self.goal_th - th)
        self.e_r = self.goal_r - r

        # error integration
        self.e_phi_int += self.e_phi*control_dt
        self.e_th_int += self.e_th*control_dt 

        # saturate integration
        self.e_phi_int = self.saturate(self.e_phi_int,-5,5)
        self.e_th_int = self.saturate(self.e_th,-5,5)

        #PID magic
        phi_in = (self.k_phi[0]*self.e_phi) +  (self.k_phi[1]*e_phi_dot) + (self.k_phi[2]*self.e_phi_i)
        th_in = (self.k_th[0]*self.e_th) +  (self.k_th[1] * e_th_dot) + (self.k_th[2]*self.e_th_i)
        r_in = self.k_r[0]*self.e_r + self.k_r[1]*e_r_dot + self.tether_tension + self.ft

        # Forces to move
        fix = -phi_in*np.sin(phi) + th_in*np.cos(th)
        fiy =  phi_in*np.cos(phi) + th_in*np.sin(th)
        fiz = -th_in*np.sin(th) 

        # TODO: Saturate fix, fiy, fiz

        #### Forces to balance ####

        [fx,fy,fz] = self.get_tether_vector(r_in,th,phi)

        #### Forces from Drag on Vehicle ####

        [fdx, fdy, fdz] = self.get_drag_vector()

        #### Total Forces for Output ####
        ftx = fix + fx + fdx
        fty = fiy + fy + fdy
        ftz = fiz + fz + fdz

        #### Rotate Forces ####
        # This is to keep the front of the vehicle pointed at the ship
        ftx = ftx*np.cos(phi) + fty*np.sin(phi)
        fty = -ftx*np.sin(phi) + fty*np.cos(phi)

        f_total = (ftx*ftx+fty*fty+ftz*ftz)**0.5
        throttle = self.thr_2_force(f_total)
        pitch = np.arctan(ftx/ftz)
        roll = np.arctan(fty/ftz)

        # Saturate
        pitch_cmd = self.saturate(pitch,-att_max,att_max)
        roll_cmd = self.saturate(roll,-att_max,att_max)


		return [roll_cmd,pitch_cmd,throttle,yaw_angle]


