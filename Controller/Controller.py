#Name: Controller.py
#Author: mklinker
#Usage: Flight scheduler and Controller (PID's and such)

import numpy as np

class Controller:
	
	def __init__(self):
		self.description = "flight scheduler and controller to handle high level commands"
		self.uavCoord = [1,1] 		#GPS coordinates of UAV [lat,lon] [degrees]
		self.uavVel = [0,0,0] 		#UAV velocity [m/s]		
		self.uavAlt = 0			#UAV Altitude [m]
		self.uavMode = 0			#UAV Mode
		self.uavAttitude = [0,0]		#Current UAV attitude (roll,pitch)[radians]

	#todo: make one function that updates all of these fields
		self.goalAttitude = [0,0]	#Goal UAV attitude (roll,pitch)[radians]	
		self.goalAlt = 0 			#Goal UAV Altitude [m]
		self.goalAngle = [0,0]		#Goal UAV Angles (theta, phi) [degrees]
	
		self.shipAlt = 0			#GPS altitude of ship station (tether) [m]
		self.shipCoord = [0,0]		#GPS position of ship station (tether) [lat,lon] [degrees]
		self.shipHeading = 0		#Heading of ship [degrees]
		self.tetherS = 0 			#Tether length [m]

		self.uavFlying = False		#Flag for if the UAV is flying or not
		self.uavFailsafe = False	#Flag for if the UAV is in failsafe or not
		self.uavArmed = False         #Flag for if the UAV is armed or not. 

	#todo: make one function that updates all of these fields
		self.relativePosition = [0,0,0] #UAV position relative to ship. (x,y,z) [m]. X is parallel with ship(forward positive), Y perpendicular to ship, Z is alt. 
		self.relativeAngle = [0,0] #UAV angles off stern of ship. (theta,phi) [degrees] theta=horizontal, phi=vertical

###############################################################################################
### Geometric Functions
###############################################################################################

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

	def getRelativeAngle(self): #Angle between stern of ship and the uav. 
		gAng = getGlobalAngle(self) #degrees
		theta = wrap360(180 - (gAng - self.shipHeading))
		phi = np.arctan( (self.uavAlt-self.shipAlt) / getDistance(self) ) *180/np.pi
		self.relativeAngle = [theta, phi]
		return self.relativeAngle 
	
	def getRelativePosition(self):
		xyDist = getDistance(self)
		z = self.uavAlt-self.shipAlt
		theta = getRelativeAngle(self) [0] 
		x = xyDist*np.cos(theta*np.pi/180)
		y = xyDist*np.sin(theta*np.pi/180)
		self.relativePosition=[x,y,z]
		return self.relativePosition

###############################################################################################
### UAV Operation Functions
###############################################################################################


	def runController(self):
		#make sure everything is updated here. 		
		
		#Calculate attitude error
		eAttitude = np.array(self.goalAttitude) - np.array(self.uavAttitude)
		
		#Calculate azmuth/inclination angle error
		eAngle = np.array(self.goalAngle) - np.array(self.relativeAngle)
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







