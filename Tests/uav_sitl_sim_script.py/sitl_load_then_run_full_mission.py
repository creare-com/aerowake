#!/usr/bin/env python2

import sys
import math
import time
import logging
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from airprobe.airprobe_run import airprobe_run

# Set connection path to vehicle
vehicle_connect_path = '127.0.0.1:14552'
uav_baud = 115200

#-------------------------------------------------------------------------------
#
# Define helper functions
#
#-------------------------------------------------------------------------------

# Test of regular guided mode waypoint command
def set_waypoint(lat,lon,altitude):
  global vehicle

  cmds = vehicle.commands
  cmds.clear()
  # lat = 37.619249
  # lon = -122.376980
  # altitude = 30.0
  cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, altitude)
  cmds.add(cmd)
  cmds.upload()

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

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

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """ 

    cmds = vehicle.commands

    print " Clear any existing commands"
    cmds.clear() 
    
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))

    print " Upload new commands to vehicle"
    cmds.upload()

def condition_yaw(heading, relative=True):
  """
  Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

  This method sets an absolute heading by default, but you can set the `relative` parameter
  to `True` to set yaw relative to the current yaw heading.

  By default the yaw of the vehicle will follow the direction of travel. After setting 
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
  msg = vehicle.message_factory.command_long_encode(
      0, 0,    # target system, target component
      mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
      0, #confirmation
      heading,    # param 1, yaw in degrees
      0,          # param 2, yaw speed deg/s
      1,          # param 3, direction -1 ccw, 1 cw
      is_relative, # param 4, relative offset 1, absolute angle 0
      0, 0, 0)    # param 5 ~ 7 not used
  # send command to vehicle
  vehicle.send_mavlink(msg)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to arm until vehicle is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
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

  # UAV (named vehicle) connection
  logging.info("Waiting for vehicle")
  while True:
    try:
      vehicle = connect(vehicle_connect_path, baud = uav_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
      break
    except OSError:
      logging.critical("Cannot find device, is the vehicle plugged in? Retrying...")
      time.sleep(5)
    except APIException:
      logging.critical("vehicle connection timed out. Retrying...")
  logging.info("vehicle connected!")

  if(vehicle.parameters['ARMING_CHECK'] != 1):
    logging.warning("vehicle reports arming checks are not standard!")

  # Bunch of seemingly necessary callbacks
  logging_time=0
  vehicle_start_time = 0
  rasp_start_time = 0
  @vehicle.on_message('SYSTEM_TIME')
  def vehicle_time_callback(self, attr_name, msg):
    global vehicle_start_time
    if(vehicle_start_time is 0 and msg.time_unix_usec > 0):
      vehicle_start_time = msg.time_unix_usec/1000000
      logging_time = "%0.4f" % time.time()
      logging.info("Got GPS lock at %s" % logging_time)
      rasp_start_time = time.clock()

  timed_out = False
  @vehicle.on_attribute('last_heartbeat')   
  def last_heartbeat_listener(self, attr_name, value):
    if(attr_name is 'last_heartbeat'):
      global timed_out
      if value > 3 and not timed_out:
        timed_out = True
        logging.critical("Pixhawk connection lost!")
      if value < 3 and timed_out:
        timed_out = False;
        logging.info("Pixhawk connection restored.")

  @vehicle.on_attribute('armed')
  def arm_disarm_callback(self,attr_name, msg):
    logging.info("Vehicle is now %sarmed " % ("" if vehicle.armed else "dis"))

  @vehicle.on_attribute('mode')
  def mode_callback(self,attr_name, mode):
    logging.info("vehicle mode changed to %s" % mode.name)

  logging.info("------------------SYSTEM IS READY!!------------------")
  logging.info("-----------------------------------------------------")

  #-----------------------------------------------------------------------------
  # Create Mission and Fly
  #-----------------------------------------------------------------------------

  print 'Create a new mission (for current location)'
  adds_square_mission(vehicle.location.global_frame,50)

  # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
  arm_and_takeoff(10)

  print "Starting mission"
  # Reset mission set to first (0) waypoint
  vehicle.commands.next=0

  # Set mode to AUTO to start mission
  vehicle.mode = VehicleMode("AUTO")

  # Monitor mission. 
  # Demonstrates getting and setting the command number 
  # Uses distance_to_current_waypoint(), a convenience function for finding the 
  #   distance to the next waypoint.

  while True:
    nextwaypoint=vehicle.commands.next
    if vehicle.mode == VehicleMode("AUTO"):
      print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
    if nextwaypoint==3: #Skip to next waypoint
      print 'Skipping to Waypoint 5 when reach waypoint 3'
      vehicle.commands.next = 5
    if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
      print 'Return to launch'
      vehicle.mode = VehicleMode("RTL")
    if not vehicle.armed:
      print "Vehicle mission terminated"
      break
    time.sleep(1)

  #Close vehicle object before exiting script
  print "Close vehicle object"
  vehicle.close()

  print 'Finished'

