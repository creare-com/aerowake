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

		# Inputs 
		self.goal_pose = [0,0,0]	# UAV Goal Position [theta,phi,r] (radians)

		# Outputs
		self.goal_attitude = [0,0] 	# UAV Goal attitude [roll, pitch] (radians)
		self.goal_throttle = 0		# UAV Goal Throttle (PWM Signal)
		self.goal_heading = 0 		# UAV Goal Heading (degrees) 
		
		# Memory Items for Control
		self.control_c = 0


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

    def pitch_2_pwm(self,angle):
        angle=angle
        return int( self.saturate(1500+(angle*500/45)  ,1000,2000))

    def roll_2_pwm(self,angle):
        angle=angle
        return int(self.saturate(1500-(angle*500/45),1000,2000))

    def thr_2_pwm(self,force):
    	thr = force*10
    	return int(self.saturate(1000+thr,1000,2000))

##!!!!!!!!!!!!!!!!!!!!!!!!!!!! Estimation Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	def get_drag_vector(self):
		return [0,0,0]

	def get_tether_vector(self):
		return [0,0,0]




##!!!!!!!!!!!!!!!!!!!!!!!!!!!! Run Controller Function !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	def run_pose_controller(self):
		self.control_c +=1


		return [roll_pwm,pitch_pwm,throttle_pwm,yaw_pwm]


