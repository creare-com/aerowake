#Name: Controller.py
#Author: mklinker
#Usage: Flight scheduler and Controller (PID's and such)

import numpy as np

class Controller:
	
	def __init__(self):
		self.description = "flight scheduler and controller to handle high level commands"
		self.uavCoord = [0,0] 		#GPS coordinates of UAV [lat,lon]
		self.uavAlt = 0			#UAV Altitude [m]
		self.uavMode = 0			#UAV Mode
		self.uavAttitude = [0,0]		#Current UAV attitude (roll,pitch)[radians]

		self.goalAttitude = [0,0]	#Goal UAV attitude (roll,pitch)[radians]	
		self.goalAlt = 0 			#Goal UAV Altitude [m]
	
		self.shipAlt = [0,0]			#GPS altitude of ship station (tether) [m]
		self.shipCoord = [0,0]		#GPS position of ship station (tether) [lat,lon]
		self.shipHeading = 0		#Heading of ship [degrees]
		self.tetherS = 0 			#Tether length [m]

		self.uavFlying = False		#Flag for if the UAV is flying or not
		self.uavFailsafe = False	#Flag for if the UAV is in failsafe or not
		
		self.relativePosition = [0,0,0] #UAV position relative to ship. (x,y,z) [m]. X is parallel with ship(forward positive), Y perpendicular to ship, Z is alt. 

	def getRelativePosition(self): #Needs to be tested -- heading angle not implemented yet
		[latShip,lonShip]=[self.shipCoord[0]*np.pi/180, self.shipCoord[1]*np.pi/180] #lat/lon in radians
		[latUav,lonUav]=[self.uavCoord[0]*np.pi/180, self.uavCoord[1]*np.pi/180]
		rShip=637100+self.shipAlt		
		rUav=637100+self.uavAlt
		poseShip=[rShip*np.cos(latShip)*np.cos(lonShip), rShip*np.cos(latShip)*np.sin(lonShip), rShip*np.sin(latShip)]
		poseUav=[rUav*np.cos(latUav)*np.cos(lonUav), rUav*np.cos(latUav)*np.sin(lonUav), rUav*np.sin(latUav)]
		[x,y,z]=poseShip-poseUav
		#rotate [x,y,z] to be aligned with the heading of the ship. 
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



	def saturate(val,lower,upper):
		if val>upper:
			val=upper
		if val<lower:
			val=lower
		return val








