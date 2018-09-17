#!/usr/bin/env python2

import logging
import numpy as np
import time
from dronekit import LocationGlobal, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil

'''
This file provides helper functions to both .../gcs/main.py and .../uav/main.py. Some functions here should only be used by the UAV (e.g. arm_and_takeoff), while others are to be used by either the UAV or the GCS (e.g. get_distance_metres). 
'''

# Setup logger
# NOTE: Logger is singleton as long as handled by the same Python interpreter. Calling logging.getLogger('logger_name') from multiple scripts on the same computer will use the same file (except in advanced cases not relevant here).
logger = logging.getLogger('uav_logger')

#-------------------------------------------------------------------------------
# Arming, Disarming, Takeoffs, and Landings
#-------------------------------------------------------------------------------

def arm_vehicle(vehicle,name):
	'''
	Arms vehicle.
	'''
	logger.info('Attempting to arm %s' %(name))
	if not vehicle.armed:
		while not vehicle.is_armable:
			logger.info(' Waiting for %s to initialise...' %(name))
			time.sleep(1)
		logger.info(' Arming motors on %s' %(name))
		vehicle.mode = VehicleMode('GUIDED')
		vehicle.armed = True
		#while not vehicle.armed:
		#	logger.info(' Waiting for %s to arm...' %(name))
		#	time.sleep(1)
		if vehicle.armed:
			logger.info(' %s armed\n' %(name))
		else:
			time.sleep(1)
			if vehicle.armed:
				logger.info(' %s armed\n' %(name))
			else:
				logger.info(' %s not recognized to be armed\n' %(name))
	else:
		logger.info(' %s is already armed\n' %(name))


def disarm_vehicle(vehicle,name):
	'''
	Disarms vehicle.
	'''
	logger.info('Attempting to disarm %s...' %(name))
	if vehicle.armed:
		vehicle.armed = False
		count = 0
		keep_trying = True
		while vehicle.armed and keep_trying:
			logger.info(' Waiting for %s to disarm...' %(name))
			time.sleep(1)
			count += 1
			if count >= 5:
				logger.warning(' WARNING: FAILED TO DISARM AFTER 5 ATTEMPTS')
				keep_trying = False
		if not vehicle.armed:
			logger.info(' %s disarmed\n' %(name))
	else:
		logger.info(' %s is already disarmed\n' %(name))


def land(vehicle,name):
	'''
	Lands vehicle.
	'''
	logger.info('%s attempting to land' %(name))
	logger.info(' Setting LAND mode on %s...' %(name))
	vehicle.mode = VehicleMode('LAND')
	# Copter vehicle will automatically disarm after so many seconds of inactivity
	while vehicle.armed:
		logger.info(' Waiting for %s landing confirmed...' %(name))
		time.sleep(3)
	logger.info(' %s disarmed after landing\n' %(name))


def takeoff(vehicle,name,aTargetAltitude):
	'''
	Takes off and flies to aTargetAltitude.
	'''
	logger.info('%s attempting to takeoff' %(name))
	if vehicle.armed:
		logger.info(' %s taking off' %(name))
		vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
		# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command after vehicle.simple_takeoff will execute immediately).
		while vehicle.location.global_relative_frame.alt <= aTargetAltitude - 1: #Trigger just below target alt.
			logger.info(' Altitude: %s', vehicle.location.global_relative_frame.alt)
			time.sleep(1)
		logger.info(' %s reached target altitude\n' %(name))
	else: 
		logger.info(' %s is not armed. Cannot takeoff.\n' %(name))


def takeoff_nogps(vehicle,name,duration):
	'''
	Takes off and flies to aTargetAltitude.
	'''
	logger.info('%s attempting to takeoff with no GPS' %(name))
	if vehicle.armed:
		logger.info(' %s taking off with no GPS' %(name))
		for i in range(0,10):
			set_attitude(vehicle, 0, 0, 0, 0.6)
			time.sleep(1)
		logger.info(' %s reached target altitude.\n' %(name))
	else: 
		logger.info(' %s is not armed. Cannot takeoff.\n' %(name))



#-------------------------------------------------------------------------------
# GPS Calculations
#-------------------------------------------------------------------------------

def get_bearing(aLocation1, aLocation2):
	'''
	Returns the bearing from location 1 to location 2, where each is a LocationGlobal object.

	https://www.movable-type.co.uk/scripts/latlong.html
	''' 
	lat1 = aLocation1.lat*np.pi/180.0 # [rad]
	lon1 = aLocation1.lon*np.pi/180.0
	lat2 = aLocation2.lat*np.pi/180.0
	lon2 = aLocation2.lon*np.pi/180.0
	dlon = lon2 - lon1
	x = np.sin(dlon)*np.cos(lat2)
	y = np.cos(lat1)*np.sin(lat2) - np.sin(lat1)*np.cos(lat2)*np.cos(dlon)
	bearing = np.arctan2(x, y)*180.0/np.pi
	if bearing < 0:
		bearing += 360.0
	logger.debug('BEARING,%s',bearing)
	return bearing


def get_distance_metres(aLocation1, aLocation2, method = 'Haversine'):
	'''
	Returns the ground distance in metres between two LocationGlobal objects. The default method uses the Haversine formula. This function also accepts an argument of 'ArduPilot' to use the ArduPilot default. 

	Haversine Method:
		https://en.wikipedia.org/wiki/Haversine_formula

	ArduPilot Method:
		This method is not nearly as accurate as the Haversine method. Using the following points, it is off by more than 30 percent.
		pt1 = LocationGlobal(42.356796, -71.098244, 10)
		pt2 = LocationGlobal(42.355629, -71.101989, 10)

		This method is an approximation, and will not be accurate over large distances and close to the earth's poles. It comes from the ArduPilot test code: https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	'''
	lat1 = aLocation1.lat*np.pi/180.0 # [rad]
	lon1 = aLocation1.lon*np.pi/180.0
	lat2 = aLocation2.lat*np.pi/180.0
	lon2 = aLocation2.lon*np.pi/180.0
	dlat = lat2 - lat1
	dlon = lon2 - lon1
	if method == 'Haversine':
		R = 6378137.0 # [m] Radius of 'spherical' earth
		a = np.sin(dlat/2.0)**2 + np.cos(lat1)*np.cos(lat2)*np.sin(dlon/2.0)**2
		c = 2*np.arctan2(np.sqrt(a), np.sqrt(1 - a))
		return R*c
	elif method == 'ArduPilot':
		dlat = dlat*180.0/np.pi # [deg]
		dlon = dlon*180.0/np.pi
		return np.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5
	else: 
		raise Exception('Invalid distance method passed.')


def get_home_location(vehicle):
	'''
	Returns a LocationGlobal object containing the vehicle's home location.

	In order to get the vehicle's onboard home location, you must first download the commands. This is becuase Pixhawk is awful (i.e. because the home position is silently stored as the 0th waypoint).
	'''
	cmds = vehicle.commands
	cmds.download()
	cmds.wait_ready()
	return vehicle.home_location


def get_location_metres(original_location, dNorth, dEast):
	'''
	RETURNS A LOCATION OBJECT OF SAME TYPE AS ORIGINAL LOCATION. THE RETURNED LOCATION OBJECT IS dNorth METERS AND dEast METERS NORTH AND EAST OF original_location. THE ALTITUDE OF THE RETURNED OBJECT IS THE SAME AS original_location. 

	Returns a location object of the same type as the original location. E.g. if original_location is LocationGlobalRelative, then a LocationGlobalRelative is returned. The returned location object contains the latitude/longitude 'dNorth' and 'dEast' metres from the specified 'original_location'. The returned location object has the same 'alt' value as 'original_location'.

	The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

	For more information see:
	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	'''
	earth_radius = 6378137.0 # [m] Radius of 'spherical' earth
	# Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*np.cos(np.pi*original_location.lat/180))
	# New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/np.pi)
	newlon = original_location.lon + (dLon * 180/np.pi)
	if type(original_location) is LocationGlobal:
		targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
	elif type(original_location) is LocationGlobalRelative:
		targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
	else:
		raise Exception('Invalid Location object passed.')
	return targetlocation



#-------------------------------------------------------------------------------
# Movement Commands - Wrappers
#-------------------------------------------------------------------------------

def goto(vehicle, dNorth, dEast, gotoFunction = None):
	'''
	COMMANDS VEHICLE TO A POSITION dNorth AND dEast RELATIVE TO ITS CURRENT POSITION. IF gotoFunction == 1, THEN IT COMMANDS A POSITION USING goto_position_target_global_int. IF not gotoFunction == 1, THEN IT COMMANDS A POSITION USING simple_goto. THESE TWO FUNCTIONS ARE EFFECTIVELY THE SAME.

	Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

	The method takes a function pointer argument with a single 'dronekit.lib.LocationGlobal' parameter for the target position. This allows it to be called with different position-setting commands. By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

	The method reports the distance to target every two seconds.
	'''

	if gotoFunction == 1:
		use_simple_goto = False
		gotoFunction = goto_position_target_global_int
	else:
		use_simple_goto = True
		gotoFunction = vehicle.simple_goto

	currentLocation = vehicle.location.global_relative_frame
	targetLocation = get_location_metres(currentLocation, dNorth, dEast)
	targetDistance = get_distance_metres(currentLocation, targetLocation)
	if use_simple_goto:
		gotoFunction(targetLocation)
	else: 
		gotoFunction(vehicle,targetLocation)

	# print 'DEBUG: targetLocation: %s' % targetLocation
	# print 'DEBUG: targetDistance: %s' % targetDistance

	# Set the following conditional to True if you want to see distance to target printed in the terminal. Note: if this is True, then the goto command will not exit until this loop ends, which means any conditional commands after this goto command will have no effect. 
	if False:
		while vehicle.mode.name == 'GUIDED': # Stop action if we are no longer in guided mode.
			# print 'DEBUG: mode: %s' %(vehicle.mode.name)
			remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
			logger.info(' Distance to target: ', remainingDistance)
			if remainingDistance <= 1: # Let 'Reached target' trigger at 1 m from target
				logger.info(' Reached target')
				break
			time.sleep(2)


def goto_reference(vehicle, referenceLocation, dNorth, dEast, dDown):
	'''
	COMMANDS VEHICLE TO A POSITION dNorth, dEast, AND dDown METERS RELATIVE TO THE SPECIFIED referenceLocation. 

	THIS COMMAND RETURNS IMMEDIATELY; IT DOES NOT WAIT FOR THE VEHICLE TO REACH THE DESTINATION. IT IS ASSUMED THAT WHATEVER CALLS THIS FUNCTION WILL HANDLE THIS CORRECTLY. THIS IS DONE TO ALLOW MORE CONTROL IN uav/main.py.
	'''

	# We must have a LocationGlobal object here, so raise an exception if not
	if type(referenceLocation) is not LocationGlobal:
		raise Exception('The goto_reference function requires a LocationGlobal object.')

	targetLocation = get_location_metres(referenceLocation, dNorth, dEast)
	'''
	NOTE: dDown will be negative to indicate a positive altitude. Thus, we must subtract dDown here.
	
	E.g.: 
		GCS Altitude: 161 meters (AMSL at Hanover, NH)
		dDown: -10 meters (desire to fly UAV 10 meters above GCS altitude)
		Target Altitude: 171 meters = gcs_alt - dDown
	'''
	targetLocation.alt = referenceLocation.alt - dDown
	goto_reference_helper(vehicle,targetLocation)

	# Set the following conditional to True if you want to see distance to target printed in the terminal. Note: if this is True, then the goto command will not exit until this loop ends, which means any conditional commands after this goto command will have no effect. 
	if False:
		while vehicle.mode.name == 'GUIDED': # Stop action if we are no longer in guided mode.
			# print 'DEBUG: mode: %s' %(vehicle.mode.name)
			remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
			logger.info(' Distance to target: ', remainingDistance)
			if remainingDistance <= 1: # Let 'Reached target' trigger at 1 m from target
				logger.info(' Reached target')
				break
			time.sleep(2)



#-------------------------------------------------------------------------------
# Navigation Commands - Position Based
#-------------------------------------------------------------------------------

def goto_reference_helper(vehicle,aLocationGlobal):
	'''
	SENDS THE MavLink COMMAND TO ENACT THE POSITION TRACKING SPECIFIED BY THE got_reference FUNCTION.

	Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

	For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

	See the above link for information on the type_mask (0 = enable, 1 = ignore). 
	At time of writing, acceleration and yaw bits are ignored.
	'''
	msg = vehicle.message_factory.set_position_target_global_int_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_GLOBAL_INT, # frame
		0b0000111111111000, # type_mask (only speeds enabled)
		aLocationGlobal.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
		aLocationGlobal.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		aLocationGlobal.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
		0, # X velocity in NED frame in m/s
		0, # Y velocity in NED frame in m/s
		0, # Z velocity in NED frame in m/s
		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	# Send command to vehicle
	vehicle.send_mavlink(msg)


def goto_position_target_global_int(vehicle,aLocation):
	'''
	WHEN CALLED WITH THE goto(...) WRAPPER, THIS IS EFFECTIVELY THE SAME AS simple_goto

	Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

	For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

	See the above link for information on the type_mask (0 = enable, 1 = ignore). 
	At time of writing, acceleration and yaw bits are ignored.
	'''
	msg = vehicle.message_factory.set_position_target_global_int_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
		0b0000111111111000, # type_mask (only speeds enabled)
		aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
		aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
		0, # X velocity in NED frame in m/s
		0, # Y velocity in NED frame in m/s
		0, # Z velocity in NED frame in m/s
		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	# Send command to vehicle
	vehicle.send_mavlink(msg)


def goto_position_target_local_ned(vehicle,north,east,down):
	''' 
	COMMANDS THE VEHICLE TO A LOCATION RELATIVE TO THE VEHICLE'S HOME POSITION. 

	Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified location in the North, East, Down frame.

	It is important to remember that in this frame, positive altitudes are entered as negative 'Down' values. So if down is '10', this will be 10 metres below the home altitude.

	Starting from AC3.3 the method respects the frame setting. Prior to that the frame was ignored. For more information see: http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

	See the above link for information on the type_mask (0=enable, 1=ignore). At time of writing, acceleration and yaw bits are ignored.
	'''
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111111000, # type_mask (only positions enabled)
		north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
		0, 0, 0, # x, y, z velocity in m/s  (not used)
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	# Send command to vehicle
	vehicle.send_mavlink(msg)



#-------------------------------------------------------------------------------
# Navigation Commands - Velocity Based
#-------------------------------------------------------------------------------

def send_ned_velocity(vehicle, velocity_north, velocity_east, velocity_down, duration):
	'''
	COMMANDS VELOCITY IN THE NORTH, EAST, DOWN FRAME FOR AN AMOUNT OF TIME SPECIFIED BY DURATION. SAME AS send_global_velocity IN TERMS OF RESULT. THE ONLY DIFFERENCE IS THE PARTICULAR COMMAND TYPE THAT IS ACTUALLY SENT. 

	Move vehicle in direction based on specified velocity vectors and
	for the specified duration.

	This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only velocity components (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
	
	Note that from AC3.3 the message should be re-sent every second (after about 3 seconds with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified	velocity persists until it is canceled. The code below should work on either version (sending the message multiple times does not cause problems).
	
	See the above link for information on the type_mask (0=enable, 1=ignore). At time of writing, acceleration and yaw bits are ignored.
	'''
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		velocity_north, velocity_east, velocity_down, # NED velocity in m/s
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	# Send command to vehicle on 1 Hz cycle
	for x in range(0,duration):
		vehicle.send_mavlink(msg)
		time.sleep(1)


def send_global_velocity(vehicle, velocity_north, velocity_east, velocity_down, duration):
	'''
	COMMANDS VELOCITY IN THE NORTH, EAST, DOWN FRAME FOR AN AMOUNT OF TIME SPECIFIED BY DURATION. SAME AS send_ned_velocity IN TERMS OF RESULT. THE ONLY DIFFERENCE IS THE PARTICULAR COMMAND TYPE THAT IS ACTUALLY SENT. 

	Move vehicle in direction based on specified velocity vectors.

	This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only velocity components (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
	
	Note that from AC3.3 the message should be re-sent every second (after about 3 seconds with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified	velocity persists until it is canceled. The code below should work on either version (sending the message multiple times does not cause problems).
	
	See the above link for information on the type_mask (0=enable, 1=ignore). At time of writing, acceleration and yaw bits are ignored.
	'''
	msg = vehicle.message_factory.set_position_target_global_int_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, # lat_int - X Position in WGS84 frame in 1e7 * meters
		0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
		# altitude above terrain if GLOBAL_TERRAIN_ALT_INT
		velocity_north, # north velocity in NED frame in m/s
		velocity_east, # east velocity in NED frame in m/s
		velocity_down, # down velocity in NED frame in m/s
		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	# Send command to vehicle on 1 Hz cycle
	for x in range(0,duration):
		vehicle.send_mavlink(msg)
		time.sleep(1)    



#-------------------------------------------------------------------------------
# Navigation Commands - Attitude Based (for GUIDED_NOGPS mode)
#-------------------------------------------------------------------------------

def set_attitude(vehicle, roll, pitch, yaw_rate, thrust):
	"""
	SETS ATTITUDE TARGET FOR GUIDED_NOGPS MODE. 
	roll, pitch [deg]
	yaw_rate [rad/s]
	thrust [normalized from 0 to 1]

	Note that from AC3.3 the message should be re-sent every second (after about 3 seconds with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified velocity persists until it is canceled. The code below should work on either version (sending the message multiple times does not cause problems).

	The roll and pitch rate cannot be controlled with rate in radian in AC3.4.4 or earlier, so you must use quaternion to control the pitch and roll for those vehicles.
	"""
	
	# Thrust >  0.5: Ascend
	# Thrust == 0.5: Hold the altitude
	# Thrust <  0.5: Descend
	msg = vehicle.message_factory.set_attitude_target_encode(
		0,
		0,                                         # target system
		0,                                         # target component
		0b00000000,                                # type mask: bit 1 is LSB
		to_quaternion(roll, pitch),    # q
		0,                                         # body roll rate in radian
		0,                                         # body pitch rate in radian
		np.radians(yaw_rate),                    # body yaw rate in radian
		thrust)                                    # thrust

	vehicle.send_mavlink(msg)


def maintain_altitude(vehicle):
	if vehicle.armed:
		set_attitude(vehicle, 0, 0, 0, 0.5)
	else: 
		logger.info(' %s is not armed. Cannot maintain altitude.\n' %(name))




#-------------------------------------------------------------------------------
# Condition Commands
#-------------------------------------------------------------------------------

def condition_yaw(vehicle, heading, relative = False):
	'''
	SETS YAW. IF relative == False, YAW WILL BE SET WITH NORTH = 0, EAST = 90, ETC. IF relative == True, YAW WILL CHANGE WITH RESPECT TO THE CURRENT HEADING WITH +CW AND -CCW. IN ORDER FOR THIS COMMAND TO WORK, AT LEAST ONE NAVIGATION COMMAND MUST HAVE BEEN GIVEN SINCE VEHICLE ENTERED GUIDED MODE. 

	Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

	This method sets an absolute heading by default, but you can set the `relative` parameter	to `True` to set yaw relative to the current yaw heading.

	By default the yaw of the vehicle will follow the direction of travel. After setting the yaw using this function there is no way to return to the default yaw 'follow direction of travel' behaviour (https://github.com/diydrones/ardupilot/issues/2427)

	For more information see: 
	http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
	'''
	if relative:
		is_relative = 1 # Yaw relative to direction of travel
		if heading > 0:
			yaw_direction = 1
		else:
			yaw_direction = -1
			# Heading must be positive
			heading = -heading
	else:
		is_relative = 0 # Yaw is an absolute angle
		yaw_direction = 1
	# Create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		heading,    # param 1, yaw in degrees
		0,          # param 2, yaw speed deg/s
		yaw_direction, # param 3, direction -1 ccw, 1 cw
		is_relative, # param 4, relative offset 1, absolute angle 0
		0, 0, 0)    # param 5 ~ 7 not used
	# Send command to vehicle
	vehicle.send_mavlink(msg)


def set_roi(vehicle,location):
	'''
	COMMANDS VEHICLE TO POINT AT THE SPECIFIED GPS LOCATION

	Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a specified region of interest (LocationGlobal). The vehicle may also turn to face the ROI.

	For more information see: 
	http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
	'''
	# Create the MAV_CMD_DO_SET_ROI command
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
		0, #confirmation
		0, 0, 0, 0, #params 1-4
		location.lat,
		location.lon,
		location.alt
		)
	# Send command to vehicle
	vehicle.send_mavlink(msg)



#-------------------------------------------------------------------------------
# Miscellaneous Commands
#-------------------------------------------------------------------------------

def emergency_stop(vehicle, name):
	'''
	IMMEDIATELY DISARMS MOTORS. DRONE FALLS OUT OF SKY.
	'''

	logger.critical('Killing %s' %(name))

	msg = vehicle.message_factory.command_long_encode(
		0, 0, # target system, target component
		mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, # command
		0, # confirmation
		1, 0, 0, 0, 0, 0, 0) # param 2 - 7 not used
	# Send command to vehicle
	vehicle.send_mavlink(msg)

	while vehicle.armed:
		pass


def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = np.cos(np.radians(yaw * 0.5))
    t1 = np.sin(np.radians(yaw * 0.5))
    t2 = np.cos(np.radians(roll * 0.5))
    t3 = np.sin(np.radians(roll * 0.5))
    t4 = np.cos(np.radians(pitch * 0.5))
    t5 = np.sin(np.radians(pitch * 0.5))
    
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    
    return [w, x, y, z]
