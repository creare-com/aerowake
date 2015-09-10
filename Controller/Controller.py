#Name: Controller.py
#Author: mklinker
#Usage: Flight scheduler and Controller (PID's and such)

import numpy as np

class Controller:
	
	def __init__(self):
		self.description = "flight scheduler and controller to handle high level commands"
		self.uav_coord = [1,1] 		#GPS coordinates of UAV [lat,lon] [degrees]
		self.uav_vel = [0,0,0] 		#UAV velocity [m/s]		
		self.uav_alt = 0			#UAV Altitude [m]
		self.uav_mode = 0			#UAV Mode
		self.uav_attitude = [0,0]		#Current UAV attitude (roll,pitch)[radians]

	#todo: make one function that updates all of these fields
		self.goal_attitude = [0,0]	#Goal UAV attitude (roll,pitch)[radians]	
		self.goal_alt = 0 			#Goal UAV Altitude [m]
		self.goal_angle = [0,0]		#Goal UAV Angles (theta, phi) [degrees]
	
		self.ship_alt = 0			#GPS altitude of ship station (tether) [m]
		self.ship_coord = [0,0]		#GPS position of ship station (tether) [lat,lon] [degrees]
		self.ship_heading = 0		#Heading of ship [degrees]
		self.ship_tether_length = 0 			#Tether length [m]

		self.uav_flying = False		#Flag for if the UAV is flying or not
		self.uav_failsafe = False	#Flag for if the UAV is in failsafe or not
		self.uav_armed = False      #Flag for if the UAV is armed or not. 
		self.uav_write_priv = False #Flag for whether the controller can write to RC Override or not. 
		
	#todo: make one function that updates all of these fields
		self.relative_position = [0,0,0] #UAV position relative to ship. (x,y,z) [m]. X is parallel with ship(forward positive), Y perpendicular to ship, Z is alt. 
		self.relative_angle = [0,0] #UAV angles off stern of ship. (theta,phi) [degrees] theta=horizontal, phi=vertical

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
		xy_dist = get_distance(self)
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

	def get_relative_angle(self): #Angle between stern of ship and the uav. 
		g_ang = get_global_angle(self) #degrees
		theta = wrap360(180 - (g_ang - self.ship_heading))
		phi = np.arctan( (self.uav_alt-self.ship_alt) / get_distance(self) ) *180/np.pi
		self.relative_angle = [theta, phi]
		return self.relative_angle 
	
	def get_relative_position(self):
		xy_dist = get_distance(self)
		z = self.uav_alt-self.ship_alt
		theta = get_relative_angle(self) [0] 
		x = xy_dist*np.cos(theta*np.pi/180)
		y = xy_dist*np.sin(theta*np.pi/180)
		self.relative_position=[x,y,z]
		return self.relative_position

###############################################################################################
### UAV Operation Functions
###############################################################################################


	def run_controller(self):
		#make sure everything is updated here. 		
		
		#Calculate attitude error
		e_attitude = np.array(self.goal_attitude) - np.array(self.uav_attitude)
		
		#Calculate azmuth/inclination angle error
		e_angle = np.array(self.goal_angle) - np.array(self.relative_angle)
		#Calculate yaw errors OR command yaw to be aligned with ship

		#make the errors as part of self??
		

		#hand back [rollPWM, pitchPWM, throttlePWM, yawPWM]		
		return [0,0,0,0] #[ch1,ch2,ch3,ch4]

###############################################################################################
### Utility Functions
###############################################################################################

	def subtrList(list1,list2): ##This can be deleted... not needed. Use np.array() instead
		ln1 = len(list1)
		ln2 = len(list2)
		if ln1 == ln2:
			outList=[0]*ln1
			for i in range(0,ln1):
				print i
				outList[i]=list1[i]-list2[i]
		else:
			print "subList function error-- input list sizes different"
			outList=[0]				
		return outList

	def wrap360(val):
		if val>360:
			val=val-360
		if val<360:
			val=val+360
		return val

	def saturate(val,lower,upper):
		if val>upper:
			val=upper
		if val<lower:
			val=lower
		return val

