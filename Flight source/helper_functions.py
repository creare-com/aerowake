#!/usr/bin/env python2

import numpy as np
import time
from dronekit import LocationGlobal, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil

#-------------------------------------------------------------------------------
#
# This file provides helper functions to both .../gcs/main.py and .../uav/main.py. Some functions here should only be used by the UAV (e.g. arm_and_takeoff), while others are to be used by either the UAV or the GCS (e.g. get_distance_metres)
#
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Arming, Disarming, Takeoffs, and Landings
#-------------------------------------------------------------------------------

def arm_vehicle(vehicle,name):
	'''
	Arms vehicle.
	'''
	print 'Attempting to arm %s' %(name)
	if not vehicle.armed:
		while not vehicle.is_armable:
			print ' Waiting for %s to initialise...' %(name)
			time.sleep(1)
		print ' Arming motors on %s' %(name)
		vehicle.mode = VehicleMode('GUIDED')
		vehicle.armed = True
		while not vehicle.armed:
			print ' Waiting for %s to arm...' %(name)
			time.sleep(1)
		print ' %s armed\n' %(name)
	else:
		print ' %s is already armed\n' %(name)


def disarm_vehicle(vehicle,name):
	'''
	Disarms vehicle.
	'''
	print 'Attempting to disarm %s...' %(name)
	if vehicle.armed:
		vehicle.armed = False
		count = 0
		keep_trying = True
		while vehicle.armed and keep_trying:
			print ' Waiting for %s to disarm...' %(name)
			time.sleep(1)
			count += 1
			if count >= 5:
				print ' WARNING: FAILED TO DISARM'
				keep_trying = False
		if not vehicle.armed:
			print ' %s disarmed\n' %(name)
	else:
		print ' %s is already disarmed\n' %(name)


def land(vehicle,name):
	'''
	Lands vehicle.
	'''
	print '%s attempting to land' %(name)
	print ' Setting LAND mode on %s...' %(name)
	vehicle.mode = VehicleMode('LAND')
	# Copter vehicle will automatically disarm after so many seconds of inactivity
	while vehicle.armed:
		print ' Waiting for %s landing confirmed...' %(name)
		time.sleep(3)
	print ' %s disarmed after landing\n' %(name)


def takeoff(vehicle,name,aTargetAltitude):
	'''
	Takes off and flies to aTargetAltitude.
	'''
	print '%s attempting to takeoff' %(name)
	if vehicle.armed:
		print ' %s taking off' %(name)
		vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
		# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command after vehicle.simple_takeoff will execute immediately).
		while vehicle.location.global_relative_frame.alt <= aTargetAltitude*0.95: #Trigger just below target alt.
			print ' Altitude: ', vehicle.location.global_relative_frame.alt 
			time.sleep(1)
		print ' %s reached target altitude\n' %(name)
	else: 
		print ' %s is not armed. Cannot takeoff.\n' %(name)



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
	if True:
		while vehicle.mode.name == 'GUIDED': # Stop action if we are no longer in guided mode.
			# print 'DEBUG: mode: %s' %(vehicle.mode.name)
			remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
			print ' Distance to target: ', remainingDistance
			if remainingDistance <= 1: # Let 'Reached target' trigger at 1 m from target
				print ' Reached target'
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
			print ' Distance to target: ', remainingDistance
			if remainingDistance <= 1: # Let 'Reached target' trigger at 1 m from target
				print ' Reached target'
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
	else:
		is_relative = 0 # Yaw is an absolute angle
	# Create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		heading,    # param 1, yaw in degrees
		0,          # param 2, yaw speed deg/s
		1,          # param 3, direction -1 ccw, 1 cw
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

