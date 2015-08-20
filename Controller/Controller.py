#Name: Controller.py
#Author: mklinker
#Usage: Flight scheduler and Controller (PID's and such)

import numpy as np

class Controller:
	
	def __init__(self):
		self.description = "flight scheduler and controller to handle high level commands"
		self.uavCoord = [0,0] 		#GPS coordinates of UAV [lat,lon] [degrees]
		self.uavAlt = 0			#UAV Altitude [m]
		self.uavMode = 0			#UAV Mode
		self.uavAttitude = [0,0]		#Current UAV attitude (roll,pitch)[radians]

		self.goalAttitude = [0,0]	#Goal UAV attitude (roll,pitch)[radians]	
		self.goalAlt = 0 			#Goal UAV Altitude [m]
	
		self.shipAlt = [0,0]			#GPS altitude of ship station (tether) [m]
		self.shipCoord = [0,0]		#GPS position of ship station (tether) [lat,lon] [degrees]
		self.shipHeading = 0		#Heading of ship [degrees]
		self.tetherS = 0 			#Tether length [m]

		self.uavFlying = False		#Flag for if the UAV is flying or not
		self.uavFailsafe = False	#Flag for if the UAV is in failsafe or not
		
		self.relativePosition = [0,0,0] #UAV position relative to ship. (x,y,z) [m]. X is parallel with ship(forward positive), Y perpendicular to ship, Z is alt. 

	def getDistance(self): #Tested separately. Gives the 2D GPS position distance from the ship to UAV
		act1 = self.uavCoord
		act2 = self.shipCoord

		R = 6371229
		dlat = act1[0]-act2[0]
		dlon = act1[1]-act2[1]

		a = (np.sin(dlat/2*np.pi/180))**2 + np.cos(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * (np.sin(dlon/2*np.pi/180))**2
		distance = R *2 * np.arctan2(np.sqrt(a),np.sqrt(1-a))
		return distance #[meters]

	def getDiagonalDistance(self): #Gives the ideal taut-tether length from the ship to the UAV. 
		xyDist = getDistance(self)
		zDist = self.uavAlt-self.shipAlt
		diagDist = ( xyDist**2 + zDist**2 )**0.5
		return diagDist #[meters]

	def getGlobalAngle(self): #Tested separately. Gives the heading angle from North from the ship to UAV. 
		act1 = self.uavCoord
		act2 = self.shipCoord
		dlat=act1[0]-act2[0]
		dlon=act1[1]-act2[1]
		
		arg1= np.sin(dlon*np.pi/180) * np.cos(act2[0]*np.pi/180) 
		arg2= np.cos(act1[0]*np.pi/180) * np.sin(act2[0]*np.pi/180) - np.sin(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * np.cos(dlon*np.pi/180)		
		gRelAng =  -np.arctan2( arg1, arg2)
		if gRelAng<0:
			gRelAng=gRelAng+2*np.pi
		gRelAng=gRelAng*180/np.pi
		return gRelAng #[degrees]

	def getRelativePosition(self):
		xyDist = getDistance(self)
		z = self.uavAlt-self.shipAlt
		shipHeadingAngle = getGlobalAngle(self) ##UPDATE WITH SHIPS HEADING
		tAngle = (180-shipHeadingAngle)*np.pi/180 # [Radians] #Angle off stern of ship, right (starboard side) is positive
		x = xyDist*np.cos(tAngle)
		y = xyDist*np.sin(tAngle)
		self.relativePosition=[x,y,z]
		return self.relativePosition

	def armUAV(self):
		#Usage: Arm the UAV, prepare for takeoff
		return null
	
	def takeOffUAV(self,alt):
		#Usage: Take the uav off and hover at [0,0] attitude at the specified altitude. 
		if not self.uavFlying and not self.uavFailsafe:
			self.uavFlying = True
			self.goalAlt = alt
			self.goalAttitude = [0,0]
			#Some machinery to increase the throttle and close the loop around a desired altitude. 
		return null

	def landUAV(self):
		#Usage: make the UAV flat and allow it to be reeled in. 
		return null

	def disarmUAV(self):
		#Usage: Turn motors off. Set throttle to 0, disarm autopilot. 
		return null
	 
	def runController(self):
		#Calculate position errors
		#Calculate altitude errors
		#Calculate yaw errors
		#
		
		#hand back [rollPWM, pitchPWM, throttlePWM, yawPWM]		
		return [0,0,0,0] #[ch1,ch2,ch3,ch4]

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








