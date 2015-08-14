#Name: Controller.py
#Author: mklinker
#Usage: Flight controller class


# needs to be a class

class Controller:
	
	def __init__(self):
		self.description = "flight controller to handle attitude commands and such"
		self.uavCoord = [0,0] 		#GPS coordinates of UAV [lat,lon]
		self.uavAlt = 0			#UAV Altitude [m]
		self.uavMode = 0			#UAV Mode
		self.goalAttitude = [0,0]	#Goal UAV attitude (roll,pitch)[radians]	
		self.uavAttitude = [0,0]		#Current UAV attitude (roll,pitch)[radians]
	
		self.shipAlt = [0,0]			#GPS altitude of ship station (tether) [m]
		self.shipCoord = [0,0]		#GPS position of ship station (tether) [lat,lon]
		self.tetherS = 0 			#Tether length [m]

		self.uavFlying = False		#Flag for if the UAV is flying or not
		self.uavFailsafe = False	#Flag for if the UAV is in failsafe or not











		










