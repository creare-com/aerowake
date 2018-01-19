#!/usr/bin/env python2

import sys
import math
import time
import logging
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from airprobe.airprobe_run import airprobe_run

#-------------------------------------------------------------------------------
#
# Global Parameters and Constants
#
#-------------------------------------------------------------------------------

# Set connection path to UAV
uav_connect_path = '127.0.0.1:14552'
uav_baud = 115200

# Set connection path to GCS
gcs_connect_path = '127.0.0.1:14554'
gcs_baud = 115200

#-------------------------------------------------------------------------------
#
# Define Helper Functions
#
#-------------------------------------------------------------------------------

def arm_and_takeoff(aTargetAltitude):
	"""
	Arms uav and fly to aTargetAltitude.
	"""

	print "Basic pre-arm checks"
	# Don't let the user try to arm until uav is ready
	while not uav.is_armable:
		print " Waiting for uav to initialise..."
		time.sleep(1)

	print "Arming motors"
	# Copter should arm in GUIDED mode
	uav.mode = VehicleMode("GUIDED")
	uav.armed = True

	while not uav.armed:
		print " Waiting for arming..."
		time.sleep(1)

	print "Taking off!"
	uav.simple_takeoff(aTargetAltitude) # Take off to target altitude

	# Wait until the uav reaches a safe height before processing the goto (otherwise the command 
	#  after uav.simple_takeoff will execute immediately).
	while True:
		print " Altitude: ", uav.location.global_relative_frame.alt
		if uav.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
			print "Reached target altitude"
			break
		time.sleep(1)

"""
Convenience functions for sending immediate/guided mode commands to control the Copter.

The set of commands demonstrated here include:
* MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
* MAV_CMD_DO_SET_ROI - set direction where the camera gimbal is aimed (latitude, longitude, altitude)
* MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.

The full set of available commands are listed here:
http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
"""

def condition_yaw(heading, relative=False):
	"""
	Send MAV_CMD_CONDITION_YAW message to point uav at a specified heading (in degrees).

	This method sets an absolute heading by default, but you can set the `relative` parameter
	to `True` to set yaw relative to the current yaw heading.

	By default the yaw of the uav will follow the direction of travel. After setting 
	the yaw using this function there is no way to return to the default yaw "follow direction 
	of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

	For more information see: 
	http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
	"""
	if relative:
		is_relative = 1 #yaw relative to direction of travel
	else:
		is_relative = 0 #yaw is an absolute angle
	# create the CONDITION_YAW command using command_long_encode()
	msg = uav.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		heading,    # param 1, yaw in degrees
		0,          # param 2, yaw speed deg/s
		1,          # param 3, direction -1 ccw, 1 cw
		is_relative, # param 4, relative offset 1, absolute angle 0
		0, 0, 0)    # param 5 ~ 7 not used
	# send command to uav
	uav.send_mavlink(msg)

def set_roi(location):
	"""
	Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
	specified region of interest (LocationGlobal).
	The uav may also turn to face the ROI.

	For more information see: 
	http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
	"""
	# create the MAV_CMD_DO_SET_ROI command
	msg = uav.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
		0, #confirmation
		0, 0, 0, 0, #params 1-4
		location.lat,
		location.lon,
		location.alt
		)
	# send command to uav
	uav.send_mavlink(msg)

"""
Functions to make it easy to convert between the different frames-of-reference. In particular these
make it easy to navigate in terms of "metres from the current position" when using commands that take 
absolute positions in decimal degrees.

The methods are approximations only, and may be less accurate over longer distances, and when close 
to the Earth's poles.

Specifically, it provides:
* get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
* get_distance_metres - Get the distance between two LocationGlobal objects in metres
* get_bearing - Get the bearing in degrees to a LocationGlobal
"""

def get_location_metres(original_location, dNorth, dEast):
	"""
	Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
	specified `original_location`. The returned LocationGlobal has the same `alt` value
	as `original_location`.

	The function is useful when you want to move the uav around specifying locations relative to 
	the current uav position.

	The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

	For more information see:
	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	"""
	earth_radius = 6378137.0 #Radius of "spherical" earth
	#Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

	#New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	if type(original_location) is LocationGlobal:
		targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
	elif type(original_location) is LocationGlobalRelative:
		targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
	else:
		raise Exception("Invalid Location object passed")
		
	return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
	"""
	Returns the ground distance in metres between two LocationGlobal objects.

	This method is an approximation, and will not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing(aLocation1, aLocation2):
	"""
	Returns the bearing between the two LocationGlobal objects passed as parameters.

	This method is an approximation, and may not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	""" 
	off_x = aLocation2.lon - aLocation1.lon
	off_y = aLocation2.lat - aLocation1.lat
	bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
	if bearing < 0:
		bearing += 360.00
	return bearing;

"""
Functions to move the uav to a specified position (as opposed to controlling movement by setting velocity components).

The methods include:
* goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in 
	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
* goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in 
	MAV_FRAME_BODY_NED frame
* goto - A convenience function that can use uav.simple_goto (default) or 
	goto_position_target_global_int to travel to a specific position in metres 
	North and East from the current location. 
	This method reports distance to the destination.
"""

def goto_position_target_global_int(aLocation):
	"""
	Send SET_POSITION_TARGET_GLOBAL_INT command to request the uav fly to a specified LocationGlobal.

	For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

	See the above link for information on the type_mask (0=enable, 1=ignore). 
	At time of writing, acceleration and yaw bits are ignored.
	"""
	msg = uav.message_factory.set_position_target_global_int_encode(
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
		0b0000111111111000, # type_mask (only speeds enabled)
		aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
		aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
		0, # X velocity in NED frame in m/s
		0, # Y velocity in NED frame in m/s
		0, # Z velocity in NED frame in m/s
		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	# send command to uav
	uav.send_mavlink(msg)

def goto_position_target_local_ned(north, east, down):
	""" 
	Send SET_POSITION_TARGET_LOCAL_NED command to request the uav fly to a specified 
	location in the North, East, Down frame.

	It is important to remember that in this frame, positive altitudes are entered as negative 
	"Down" values. So if down is "10", this will be 10 metres below the home altitude.

	Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
	ignored. For more information see: 
	http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

	See the above link for information on the type_mask (0=enable, 1=ignore). 
	At time of writing, acceleration and yaw bits are ignored.

	"""
	msg = uav.message_factory.set_position_target_local_ned_encode(
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111111000, # type_mask (only positions enabled)
		north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
		0, 0, 0, # x, y, z velocity in m/s  (not used)
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	# send command to uav
	uav.send_mavlink(msg)

def goto(dNorth, dEast, gotoFunction=None):
	if gotoFunction == None:
		gotoFunction=uav.simple_goto
		
	"""
	Moves the uav to a position dNorth metres North and dEast metres East of the current position.

	The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
	the target position. This allows it to be called with different position-setting commands. 
	By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

	The method reports the distance to target every two seconds.
	"""
	
	currentLocation = uav.location.global_relative_frame
	targetLocation = get_location_metres(currentLocation, dNorth, dEast)
	targetDistance = get_distance_metres(currentLocation, targetLocation)
	gotoFunction(targetLocation)
	
	#print "DEBUG: targetLocation: %s" % targetLocation
	#print "DEBUG: targetLocation: %s" % targetDistance

	while uav.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
		#print "DEBUG: mode: %s" % uav.mode.name
		remainingDistance=get_distance_metres(uav.location.global_relative_frame, targetLocation)
		print "Distance to target: ", remainingDistance
		if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
			print "Reached target"
			break;
		time.sleep(2)

"""
Functions that move the uav by specifying the velocity components in each direction.
The two functions use different MAVLink commands. The main difference is
that depending on the frame used, the NED velocity can be relative to the uav
orientation.

The methods include:
* send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
* send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
"""

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
	"""
	Move uav in direction based on specified velocity vectors and
	for the specified duration.

	This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
	velocity components 
	(http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
	
	Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
	with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
	velocity persists until it is canceled. The code below should work on either version 
	(sending the message multiple times does not cause problems).
	
	See the above link for information on the type_mask (0=enable, 1=ignore). 
	At time of writing, acceleration and yaw bits are ignored.
	"""
	msg = uav.message_factory.set_position_target_local_ned_encode(
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

	# send command to uav on 1 Hz cycle
	for x in range(0,duration):
		uav.send_mavlink(msg)
		time.sleep(1)

def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
	"""
	Move uav in direction based on specified velocity vectors.

	This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
	velocity components 
	(http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
	
	Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
	with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
	velocity persists until it is canceled. The code below should work on either version 
	(sending the message multiple times does not cause problems).
	
	See the above link for information on the type_mask (0=enable, 1=ignore). 
	At time of writing, acceleration and yaw bits are ignored.
	"""
	msg = uav.message_factory.set_position_target_global_int_encode(
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, # lat_int - X Position in WGS84 frame in 1e7 * meters
		0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
		# altitude above terrain if GLOBAL_TERRAIN_ALT_INT
		velocity_x, # X velocity in NED frame in m/s
		velocity_y, # Y velocity in NED frame in m/s
		velocity_z, # Z velocity in NED frame in m/s
		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

	# send command to uav on 1 Hz cycle
	for x in range(0,duration):
		uav.send_mavlink(msg)
		time.sleep(1)    

#-------------------------------------------------------------------------------
#
# Start main process
#
#-------------------------------------------------------------------------------

if __name__ == '__main__':

	# Log Setup
	logger = logging.getLogger()
	logger.setLevel(logging.DEBUG)
	fh = logging.FileHandler('system.log')
	fh.setLevel(logging.DEBUG)
	ch = logging.StreamHandler(sys.stdout)
	ch.setLevel(logging.DEBUG)
	form_fh = logging.Formatter('%(relativeCreated)s,%(levelname)s: %(message)s')
	form_ch = logging.Formatter('%(levelname)s: %(message)s')
	fh.setFormatter(form_fh)
	ch.setFormatter(form_ch)
	logger.addHandler(fh)
	logger.addHandler(ch)

	# UAV connection
	logging.info("Waiting for UAV")
	while True:
		try:
			uav = connect(uav_connect_path, baud = uav_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except OSError:
			logging.critical("Cannot find device, is the uav plugged in? Retrying...")
			time.sleep(5)
		except APIException:
			logging.critical("UAV connection timed out. Retrying...")
	
	logging.info("UAV connected!")

	if(uav.parameters['ARMING_CHECK'] != 1):
		logging.warning("uav reports arming checks are not standard!")

	# GCS connection
	logging.info("Waiting for GCS")
	while True:
		try:
			gcs = connect(gcs_connect_path, baud = gcs_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except OSError:
			logging.critical("Cannot find device, is the gcs plugged in? Retrying...")
			time.sleep(5)
		except APIException:
			logging.critical("GCS connection timed out. Retrying...")
	
	logging.info("GCS connected!")

	# Bunch of seemingly necessary callbacks
	logging_time=0
	uav_start_time = 0
	rasp_start_time = 0
	@uav.on_message('SYSTEM_TIME')
	def uav_time_callback(self, attr_name, msg):
		global uav_start_time
		if(uav_start_time is 0 and msg.time_unix_usec > 0):
			uav_start_time = msg.time_unix_usec/1000000
			logging_time = "%0.4f" % time.time()
			logging.info("Got GPS lock at %s" % logging_time)
			rasp_start_time = time.clock()

	timed_out = False
	@uav.on_attribute('last_heartbeat')   
	def last_heartbeat_listener(self, attr_name, value):
		if(attr_name is 'last_heartbeat'):
			global timed_out
			if value > 3 and not timed_out:
				timed_out = True
				logging.critical("Pixhawk connection lost!")
			if value < 3 and timed_out:
				timed_out = False;
				logging.info("Pixhawk connection restored.")

	@uav.on_attribute('armed')
	def arm_disarm_callback(self,attr_name, msg):
		logging.info("uav is now %sarmed " % ("" if uav.armed else "dis"))

	@uav.on_attribute('mode')
	def mode_callback(self,attr_name, mode):
		logging.info("uav mode changed to %s" % mode.name)

	logging.info("------------------SYSTEM IS READY!!------------------")
	logging.info("-----------------------------------------------------")

	#-----------------------------------------------------------------------------
	#
	# Control Code
	#
	#-----------------------------------------------------------------------------

	# NOTE: ROI and yaw commands are reset when the mode or the command used to control movement is changed.
	# The correct order is: 
	#		- move command
	#		- ROI or yaw command

	# Arm and take of to altitude of 5 meters
	# arm_and_takeoff(10)

	DURATION = 20 #Set duration for each segment.
	continue_loop = True
	while continue_loop:
		try:
			# Read GCS information
			gcs_location = gcs.location.global_frame
			gcs_param = gcs.parameters['PIVOT_TURN_ANGLE']

			# Set drone's home position equal to gcs's location
			uav.home_location = gcs_location
			print 'Set home to \n\t', uav.home_location

			# Navigate to a location 15 meters south of GCS and yaw towards GCS
			print 'Flying -15m south of home'
			goto_position_target_local_ned(15,0,-10)
			print 'Yawing towards home'
			set_roi(uav.home_location)

			cmds = uav.commands
			cmds.download()
			cmds.wait_ready()
			print "New Home Location (from vehicle - altitude should be 222): \n\t%s" % uav.home_location

			print 'Going to sleep'
			time.sleep(5)
		except KeyboardInterrupt:
			continue_loop = False

	"""
	The example is completing. LAND at current location.
	"""

	# print("Setting LAND mode...")
	# uav.mode = VehicleMode("LAND")

	# while uav.armed:
	# 	print 'Waiting for landing confirmed'
	# 	time.sleep(2)

	# print 'uav disarmed'

	#Close uav object before exiting script
	print "Close uav object"
	uav.close()

	print("Completed")
	
















































	if False:
		"""
		Fly a triangular path using the standard Vehicle.simple_goto() method.

		The method is called indirectly via a custom "goto" that allows the target position to be
		specified as a distance in metres (North/East) from the current position, and which reports
		the distance-to-target.
		"""
		print("TRIANGLE path using standard Vehicle.simple_goto()")

		print("Set groundspeed to 5m/s.")
		uav.groundspeed=5

		print("Position North 80 West 50")
		goto(80, -50)

		print("Position North 0 East 100")
		goto(0, 100)

		print("Position North -80 West 50")
		goto(-80, -50)

		"""
		Fly a triangular path using the SET_POSITION_TARGET_GLOBAL_INT command and specifying
		a target position (rather than controlling movement using velocity vectors). The command is
		called from goto_position_target_global_int() (via `goto`).

		The goto_position_target_global_int method is called indirectly from a custom "goto" that allows 
		the target position to be specified as a distance in metres (North/East) from the current position, 
		and which reports the distance-to-target.

		The code also sets the speed (MAV_CMD_DO_CHANGE_SPEED). In AC3.2.1 Copter will accelerate to this speed 
		near the centre of its journey and then decelerate as it reaches the target. 
		In AC3.3 the speed changes immediately.
		""" 

		print("TRIANGLE path using standard SET_POSITION_TARGET_GLOBAL_INT message and with varying speed.")
		print("Position South 100 West 130")

		print("Set groundspeed to 5m/s.")
		uav.groundspeed = 5
		goto(-100, -130, goto_position_target_global_int)

		print("Set groundspeed to 15m/s (max).")
		uav.groundspeed = 15
		print("Position South 0 East 200")
		goto(0, 260, goto_position_target_global_int)

		print("Set airspeed to 10m/s (max).")
		uav.airspeed = 10

		print("Position North 100 West 130")
		goto(100, -130, goto_position_target_global_int)

		"""
		Fly the uav in a 50m square path, using the SET_POSITION_TARGET_LOCAL_NED command 
		and specifying a target position (rather than controlling movement using velocity vectors). 
		The command is called from goto_position_target_local_ned() (via `goto`).

		The position is specified in terms of the NED (North East Down) relative to the Home location.

		WARNING: The "D" in NED means "Down". Using a positive D value will drive the uav into the ground!

		The code sleeps for a time (DURATION) to give the uav time to reach each position (rather than 
		sending commands based on proximity).

		The code also sets the region of interest (MAV_CMD_DO_SET_ROI) via the `set_roi()` method. This points the 
		camera gimbal at the the selected location (in this case it aligns the whole uav to point at the ROI).
		""" 

		print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and position parameters")
		DURATION = 20 #Set duration for each segment.

		print("North 50m, East 0m, 10m altitude for %s seconds" % DURATION)
		goto_position_target_local_ned(50,0,-10)
		print("Point ROI at current location (home position)") 
		# NOTE that this has to be called after the goto command as first "move" command of a particular type
		# "resets" ROI/YAW commands
		set_roi(uav.location.global_relative_frame)
		time.sleep(DURATION)

		print("North 50m, East 50m, 10m altitude")
		goto_position_target_local_ned(50,50,-10)
		time.sleep(DURATION)

		print("Point ROI at current location")
		set_roi(uav.location.global_relative_frame)

		print("North 0m, East 50m, 10m altitude")
		goto_position_target_local_ned(0,50,-10)
		time.sleep(DURATION)

		print("North 0m, East 0m, 10m altitude")
		goto_position_target_local_ned(0,0,-10)
		time.sleep(DURATION)

		"""
		Fly the uav in a SQUARE path using velocity vectors (the underlying code calls the 
		SET_POSITION_TARGET_LOCAL_NED command with the velocity parameters enabled).

		The thread sleeps for a time (DURATION) which defines the distance that will be travelled.

		The code also sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method in each segment
		so that the front of the uav points in the direction of travel
		"""

		# Set up velocity vector to map to each direction.
		# vx > 0 => fly North
		# vx < 0 => fly South
		NORTH = 2
		SOUTH = -2

		# Note for vy:
		# vy > 0 => fly East
		# vy < 0 => fly West
		EAST = 2
		WEST = -2

		# Note for vz: 
		# vz < 0 => ascend
		# vz > 0 => descend
		UP = -0.5
		DOWN = 0.5

		# Square path using velocity
		print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and velocity parameters")

		print("Yaw 180 absolute (South)")
		condition_yaw(180)

		print("Velocity South & up")
		send_ned_velocity(SOUTH,0,UP,DURATION)
		send_ned_velocity(0,0,0,1)

		print("Yaw 270 absolute (West)")
		condition_yaw(270)

		print("Velocity West & down")
		send_ned_velocity(0,WEST,DOWN,DURATION)
		send_ned_velocity(0,0,0,1)

		print("Yaw 0 absolute (North)")
		condition_yaw(0)

		print("Velocity North")
		send_ned_velocity(NORTH,0,0,DURATION)
		send_ned_velocity(0,0,0,1)

		print("Yaw 90 absolute (East)")
		condition_yaw(90)

		print("Velocity East")
		send_ned_velocity(0,EAST,0,DURATION)
		send_ned_velocity(0,0,0,1)

		"""
		Fly the uav in a DIAMOND path using velocity vectors (the underlying code calls the 
		SET_POSITION_TARGET_GLOBAL_INT command with the velocity parameters enabled).

		The thread sleeps for a time (DURATION) which defines the distance that will be travelled.

		The code sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method using relative headings
		so that the front of the uav points in the direction of travel.

		At the end of the second segment the code sets a new home location to the current point.
		"""

		print("DIAMOND path using SET_POSITION_TARGET_GLOBAL_INT and velocity parameters")
		# vx, vy are parallel to North and East (independent of the uav orientation)

		print("Yaw 225 absolute")
		condition_yaw(225)

		print("Velocity South, West and Up")
		send_global_velocity(SOUTH,WEST,UP,DURATION)
		send_global_velocity(0,0,0,1)

		print("Yaw 90 relative (to previous yaw heading)")
		condition_yaw(90,relative=True)

		print("Velocity North, West and Down")
		send_global_velocity(NORTH,WEST,DOWN,DURATION)
		send_global_velocity(0,0,0,1)

		print("Set new home location to current location")
		uav.home_location=uav.location.global_frame
		print "Get new home location"
		#This reloads the home location in DroneKit and GCSs
		cmds = uav.commands
		cmds.download()
		cmds.wait_ready()
		print " Home Location: %s" % uav.home_location

		print("Yaw 90 relative (to previous yaw heading)")
		condition_yaw(90,relative=True)

		print("Velocity North and East")
		send_global_velocity(NORTH,EAST,0,DURATION)
		send_global_velocity(0,0,0,1)

		print("Yaw 90 relative (to previous yaw heading)")
		condition_yaw(90,relative=True)

		print("Velocity South and East")
		send_global_velocity(SOUTH,EAST,0,DURATION)
		send_global_velocity(0,0,0,1)
