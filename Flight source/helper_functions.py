#!/usr/bin/env python2

import time
from dronekit import LocationGlobal, LocationGlobalRelative, VehicleMode

#-------------------------------------------------------------------------------
#
# This file provides helper functions to both .../gcs/main.py and .../uav/main.py. Some functions here should only be used by the UAV (e.g. arm_and_takeoff), while others are to be used by either the UAV or the GCS (e.g. get_distance_metres)
#
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Arming and Disarming
#-------------------------------------------------------------------------------

def arm_vehicle(vehicle,name):
	"""
	Arms vehicle.
	"""
	if not vehicle.armed:
		print "Basic pre-arm checks"
		while not vehicle.is_armable:
			print " Waiting for %s to initialise..." %(name)
			time.sleep(1)
		print "Arming motors"
		vehicle.mode = VehicleMode("GUIDED")
		vehicle.armed = True
		while not vehicle.armed:
			print " Waiting for arming..."
			time.sleep(1)
		print "%s armed." %(name)
	else:
		print '%s already armed.' %(name)


def arm_and_takeoff(vehicle,aTargetAltitude,name):
	"""
	Arms vehicle and flies to aTargetAltitude.
	"""
	print "Basic pre-arm checks"
	# Don't let the user try to arm until uav is ready
	while not vehicle.is_armable:
		print " Waiting for %s to initialise..." %(name)
		time.sleep(1)
	print "Arming motors"
	# Vehicle should arm in GUIDED mode
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	while not vehicle.armed:
		print " Waiting for arming..."
		time.sleep(1)
	print "Taking off!"
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
	# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command after vehicle.simple_takeoff will execute immediately).
	while vehicle.location.global_relative_frame.alt <= aTargetAltitude*0.95: #Trigger just below target alt.
		print " Altitude: ", vehicle.location.global_relative_frame.alt 
		time.sleep(1)
	print "Reached target altitude"


def disarm_vehicle(vehicle,name):
	"""
	Disarms vehicle.
	"""
	if vehicle.armed:
		vehicle.armed = False
		while vehicle.armed:
			print " Waiting for disarming..."
			time.sleep(1)
		print "%s disarmed." %(name)
	else:
		print '%s already disarmed.' %(name)



#-------------------------------------------------------------------------------
# GPS Calculations
#-------------------------------------------------------------------------------

def get_bearing(aLocation1, aLocation2):
	"""
	Returns the bearing from location 1 to location 2, where each is a LocationGlobal object.

	https://www.movable-type.co.uk/scripts/latlong.html
	""" 
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
	"""
	Returns the ground distance in metres between two LocationGlobal objects. The default method uses the Haversine formula. This function also accepts an argument of 'ArduPilot' to use the ArduPilot default. 

	Haversine Method:
		https://en.wikipedia.org/wiki/Haversine_formula

	ArduPilot Method:
		This method is not nearly as accurate as the Haversine method. Using the following points, it is off by more than 30 percent.
		pt1 = LocationGlobal(42.356796, -71.098244, 10)
		pt2 = LocationGlobal(42.355629, -71.101989, 10)

		This method is an approximation, and will not be accurate over large distances and close to the earth's poles. It comes from the ArduPilot test code: https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""
	lat1 = aLocation1.lat*np.pi/180.0 # [rad]
	lon1 = aLocation1.lon*np.pi/180.0
	lat2 = aLocation2.lat*np.pi/180.0
	lon2 = aLocation2.lon*np.pi/180.0
	dlat = lat2 - lat1
	dlon = lon2 - lon1
	if method == 'Haversine':
		R = 6378137.0 # [m] Radius of "spherical" earth
		a = np.sin(dlat/2.0)**2 + np.cos(lat1)*np.cos(lat2)*np.sin(dlon/2.0)**2
		c = 2*np.arctan2(np.sqrt(a), np.sqrt(1 - a))
		return R*c
	elif method == 'ArduPilot':
		dlat = dlat*180.0/np.pi # [deg]
		dlon = dlon*180.0/np.pi
		return math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5
	else: 
		raise Exception("Invalid distance method passed.")


def get_location_metres(original_location, dNorth, dEast):
	"""
	Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the specified `original_location`. The returned LocationGlobal has the same `alt` value as `original_location`.

	The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

	For more information see:
	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	"""
	earth_radius = 6378137.0 # [m] Radius of "spherical" earth
	# Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
	# New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	if type(original_location) is LocationGlobal:
		targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
	elif type(original_location) is LocationGlobalRelative:
		targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
	else:
		raise Exception("Invalid Location object passed.")
	return targetlocation

#-------------------------------------------------------------------------------
# Movement Commands - Position Based
#-------------------------------------------------------------------------------





#-------------------------------------------------------------------------------
# Movement Commands - Velocity Based
#-------------------------------------------------------------------------------








