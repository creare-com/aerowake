#!/usr/bin/env python2

import sys
sys.path.append('../')

import logging
import mission
import time

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from helper_functions import arm_vehicle, condition_yaw, disarm_vehicle, get_distance_metres, get_home_location, get_location_metres, goto, goto_position_target_local_ned, land, send_global_velocity, send_ned_velocity, set_roi, takeoff

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

# Determine number of waypoints
wp_N = mission.wp_N
wp_E = mission.wp_N
wp_D = mission.wp_N
num_wp = mission.num_wp

#-------------------------------------------------------------------------------
#
# Start Main Process
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
	logging.info('Waiting for UAV')
	while True:
		try:
			uav = connect(uav_connect_path, baud = uav_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except OSError:
			logging.critical('Cannot find device, is the UAV plugged in? Retrying...')
			time.sleep(5)
		except APIException:
			logging.critical('UAV connection timed out. Retrying...')
	
	logging.info('UAV pixhawk connected to UAV')

	if(uav.parameters['ARMING_CHECK'] != 1):
		logging.warning('UAV reports arming checks are not standard!')

	# GCS connection
	logging.info('Waiting for GCS')
	while True:
		try:
			gcs = connect(gcs_connect_path, baud = gcs_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except OSError:
			logging.critical('Cannot find device, is the GCS plugged in? Retrying...')
			time.sleep(5)
		except:
			logging.critical('GCS connection timed out. Retrying...')
	
	logging.info('GCS pixhawk connected to UAV')

	# Bunch of seemingly necessary callbacks
	logging_time=0
	uav_start_time = 0
	rasp_start_time = 0
	@uav.on_message('SYSTEM_TIME')
	def uav_time_callback(self, attr_name, msg):
		global uav_start_time
		if(uav_start_time is 0 and msg.time_unix_usec > 0):
			uav_start_time = msg.time_unix_usec/1000000
			logging_time = '%0.4f' % time.time()
			logging.info('UAV got GPS lock at %s' % logging_time)
			rasp_start_time = time.clock()

	timed_out = False
	@uav.on_attribute('last_heartbeat')   
	def last_heartbeat_listener(self, attr_name, value):
		if(attr_name is 'last_heartbeat'):
			global timed_out
			if value > 3 and not timed_out:
				timed_out = True
				logging.critical('UAV pixhawk connection lost!')
			if value < 3 and timed_out:
				timed_out = False;
				logging.info('UAV pixhawk connection restored.')

	@uav.on_attribute('armed')
	def arm_disarm_callback(self,attr_name, msg):
		logging.info('UAV is now %sarmed ' % ('' if uav.armed else 'dis'))

	@uav.on_attribute('mode')
	def mode_callback(self,attr_name, mode):
		logging.info('UAV mode changed to %s' % mode.name)

	logging.info('------------------SYSTEM IS READY!!------------------')
	logging.info('-----------------------------------------------------\n')

	#-----------------------------------------------------------------------------
	#
	# Control Code
	#
	#-----------------------------------------------------------------------------

	#------------------------------------
	# Initialization
	#------------------------------------

	# Arm and take of to altitude of 5 meters
	# arm_vehicle(uav,'UAV')
	# takeoff(uav,'UAV',10)

	# Command vehicle to stay where it is. This is done so that any condition command will be enacted when it is supposed to be. Pixhawk will not enact a condition command (yaw/ROI) until the first position command has been given. 
	# send_ned_velocity(uav,0,0,0,1)

	#------------------------------------
	# Controlling
	#------------------------------------

	'''
	This while loop performs the following functions:
		- read GCS position
		- read GCS parameter value of PIVOT_TURN_ANGLE to determine next UAV action
		- perform an action based upon the GCS parameter value
	The parameter values corresponding actions to be performed are:
	 	100		UAV will follow previous command, or do nothing if no command sent yet
	 	101		UAV will begin listening to these commands
	 	359		UAV will arm
	 	358		UAV will disarm
	 	357		UAV will takeoff to a pre-programmed height and relative position
	 	356		UAV will land according to its landing protocol
	 	0+		UAV will navigate to the waypoint at the index specified
	 					The acceptable waypoint indices are 0 through num_wp - 1
	'''

	current_wp = None

	command = 100 # Command is an echo for param that disallows repeated commands
	in_the_air = False
	uav_height = uav.location.global_relative_frame.alt
	if uav_height > 0.5:
		in_the_air = True
		command = 357
	listening = False
	continue_loop = True
	while continue_loop:
		gcs_pos = gcs.location.global_relative_frame
		param = gcs.parameters['PIVOT_TURN_ANGLE']
		uav_height = uav.location.global_relative_frame.alt # [m] height relative to home location

		'''
		The variable 'command' is what controls the drone. It is only updated when a valid and non-repeated param value is set. This ensures that the UAV does not continue commanding the same thing over and over. 

		The 'command' variable can only be set while 'listening' is True. If the UAV is not listening to the GCS, then it will follow the most recent 'command' variable. 

		It is occasionally desireable for waypoint commands to be repeated, so waypoint commands are handled later on in the code.

		The parameter for 'stop listening' will be immediately passed through since repeated 'stop listening' commands are occasionally desired.
		'''
		if listening:
			print 'DEBUG: Listening'
			if param == 100:
				print 'DEBUG: Got stop listening command'
			elif param == 359 and not command == 359:
				print 'DEBUG: Got arm command'
				command = param
			elif param == 358 and not command == 358:
				print 'DEBUG: Got disarm command'
				command = param
			elif param == 357 and not command == 357:
				print 'DEBUG: Got takeoff command'
				command = param
			elif param == 356 and not command == 356:
				print 'DEBUG: Got land command'
				command = param
			elif param < num_wp:
				# NOTE: GCS will not allow a non-existent index to be passed. The handling of incorrect indices is included in the conditional above as a redundant safety feature. 
				if not command == param:
					print 'DEBUG: Got navigate to waypoint %d command' %(param)
					command = param
					current_wp = param
		else:
			print 'DEBUG: Not listening'
			if param == 101:
				print 'DEBUG: Got start listening command'
				listening = True

		# Do the action that corresponds to the current value of 'command'		
		if command == 100:
			print 'DEBUG: No commands have been sent. Waiting for command.'
		elif in_the_air:
			if command == 356:
				print 'DEBUG: Landing'
				land(uav,'UAV')
				in_the_air = False
				current_wp = None
			else:
				if not current_wp is None:
					print 'DEBUG: Flying to waypoint %d' %(current_wp)
					print 'NEED TO IMPLEMENT: FLY TO WAYPOINT'
				else:
					print 'DEBUG: In the air, not tracking a waypoint'
		elif not in_the_air: # Explicit for comprehension
			if command == 359 and not uav.armed:
				print 'DEBUG: Arming'
				arm_vehicle(uav,'UAV')
			elif command == 358 and uav.armed:
				print 'DEBUG: Disarming'
				disarm_vehicle(uav,'UAV')
			elif command == 357 and uav.armed:
				print 'DEBUG: Taking off'
				takeoff(uav,'UAV',10)
				in_the_air = True

		print ''
		time.sleep(1)

	#------------------------------------
	# Terminating
	#------------------------------------

	# # The example is completing. LAND at current location.
	# land(uav,'UAV')

	# # Disarm and close UAV object before exiting script
	# disarm_vehicle(uav,'UAV')
	# uav.close()

	# print('UAV program completed\n')








































	if False:
		'''
		The code in this conditional block shows examples of some of the functions available to the vehicle. These examples and the associated functions can be found at: 
			http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
		'''

		'''
		FLY RELATIVE TO CURRENT POSITION

		Fly a triangular path using the standard Vehicle.simple_goto() method.

		The method is called indirectly via a custom 'goto' that allows the target position to be	specified as a distance in metres (North/East) from the current position, and which reports	the distance-to-target.
		'''
		print('TRIANGLE path using standard Vehicle.simple_goto()')

		print('Set groundspeed to 5m/s.')
		uav.groundspeed=5

		print('Position North 80 West 50')
		goto(uav, 80, -50)

		print('Position North 0 East 100')
		goto(uav, 0, 100)

		print('Position North -80 West 50')
		goto(uav, -80, -50)

		'''
		FLY 

		Fly a triangular path using the SET_POSITION_TARGET_GLOBAL_INT command and specifying	a target position (rather than controlling movement using velocity vectors). The command is called from goto_position_target_global_int() (via `goto`).

		The goto_position_target_global_int method is called indirectly from a custom 'goto' that allows the target position to be specified as a distance in metres (North/East) from the current position, and which reports the distance-to-target.

		The code also sets the speed (MAV_CMD_DO_CHANGE_SPEED). In AC3.2.1 Copter will accelerate to this speed near the centre of its journey and then decelerate as it reaches the target. In AC3.3 the speed changes immediately.
		''' 

		print('TRIANGLE path using standard SET_POSITION_TARGET_GLOBAL_INT message and with varying speed.')
		print('Position South 100 West 130')

		print('Set groundspeed to 5m/s.')
		uav.groundspeed = 5
		goto(uav, -100, -130, 1)

		print('Set groundspeed to 15m/s (max).')
		uav.groundspeed = 15
		print('Position South 0 East 200')
		goto(uav, 0, 260, 1)

		print('Set airspeed to 10m/s (max).')
		uav.airspeed = 10

		print('Position North 100 West 130')
		goto(uav, 100, -130, 1)

		'''
		Fly the uav in a 50m square path, using the SET_POSITION_TARGET_LOCAL_NED command and specifying a target position (rather than controlling movement using velocity vectors). The command is called from goto_position_target_local_ned() (via `goto`).

		The position is specified in terms of the NED (North East Down) relative to the Home location.

		WARNING: The 'D' in NED means 'Down'. Using a positive D value will drive the uav into the ground!

		The code sleeps for a time (DURATION) to give the uav time to reach each position (rather than sending commands based on proximity).

		The code also sets the region of interest (MAV_CMD_DO_SET_ROI) via the `set_roi()` method. This points the camera gimbal at the the selected location (in this case it aligns the whole uav to point at the ROI).
		''' 

		print('SQUARE path using SET_POSITION_TARGET_LOCAL_NED and position parameters')
		DURATION = 20 #Set duration for each segment.

		print('North 50m, East 0m, 10m altitude for %s seconds' % DURATION)
		goto_position_target_local_ned(uav, 50, 0, -10)
		print('Point ROI at current location (home position)') 
		# NOTE that this has to be called after the goto command as first 'move' command of a particular type
		# 'resets' ROI/YAW commands
		set_roi(uav, uav.location.global_relative_frame)
		time.sleep(DURATION)

		print('North 50m, East 50m, 10m altitude')
		goto_position_target_local_ned(uav, 50,50,-10)
		time.sleep(DURATION)

		print('Point ROI at current location')
		set_roi(uav, uav.location.global_relative_frame)

		print('North 0m, East 50m, 10m altitude')
		goto_position_target_local_ned(uav, 0,50,-10)
		time.sleep(DURATION)

		print('North 0m, East 0m, 10m altitude')
		goto_position_target_local_ned(uav, 0,0,-10)
		time.sleep(DURATION)

		'''
		Fly the uav in a SQUARE path using velocity vectors (the underlying code calls the SET_POSITION_TARGET_LOCAL_NED command with the velocity parameters enabled).

		The thread sleeps for a time (DURATION) which defines the distance that will be travelled.

		The code also sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method in each segment so that the front of the uav points in the direction of travel
		'''

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
		print('SQUARE path using SET_POSITION_TARGET_LOCAL_NED and velocity parameters')

		print('Yaw 180 absolute (South)')
		condition_yaw(uav,180)

		print('Velocity South & up')
		send_ned_velocity(uav, SOUTH,0,UP,DURATION)
		send_ned_velocity(uav, 0,0,0,1)

		print('Yaw 270 absolute (West)')
		condition_yaw(uav,270)

		print('Velocity West & down')
		send_ned_velocity(uav, 0,WEST,DOWN,DURATION)
		send_ned_velocity(uav, 0,0,0,1)

		print('Yaw 0 absolute (North)')
		condition_yaw(uav,0)

		print('Velocity North')
		send_ned_velocity(uav, NORTH,0,0,DURATION)
		send_ned_velocity(uav, 0,0,0,1)

		print('Yaw 90 absolute (East)')
		condition_yaw(uav,90)

		print('Velocity East')
		send_ned_velocity(uav, 0,EAST,0,DURATION)
		send_ned_velocity(uav, 0,0,0,1)

		'''
		Fly the uav in a DIAMOND path using velocity vectors (the underlying code calls the SET_POSITION_TARGET_GLOBAL_INT command with the velocity parameters enabled).

		The thread sleeps for a time (DURATION) which defines the distance that will be travelled.

		The code sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method using relative headings so that the front of the uav points in the direction of travel.

		At the end of the second segment the code sets a new home location to the current point.
		'''

		print('DIAMOND path using SET_POSITION_TARGET_GLOBAL_INT and velocity parameters')
		# vx, vy are parallel to North and East (independent of the uav orientation)

		print('Yaw 225 absolute')
		condition_yaw(uav,225)

		print('Velocity South, West and Up')
		send_global_velocity(uav, SOUTH,WEST,UP,DURATION)
		send_global_velocity(uav, 0,0,0,1)

		print('Yaw 90 relative (to previous yaw heading)')
		condition_yaw(uav,90,relative=True)

		print('Velocity North, West and Down')
		send_global_velocity(uav, NORTH,WEST,DOWN,DURATION)
		send_global_velocity(uav, 0,0,0,1)

		# # NOTE: SETTING HOME POSITION DOES NOT WORK. NO METHOD TO SET HOME POSITION HAS SUCCEEDED.
		# print('Set new home location to current location')
		# uav.home_location = uav.location.global_frame
		# print 'Get new home location'
		# #This reloads the home location in DroneKit and GCSs
		# cmds = uav.commands
		# cmds.download()
		# cmds.wait_ready()
		# print ' Home Location: %s' % uav.home_location

		print('Yaw 90 relative (to previous yaw heading)')
		condition_yaw(uav,90,relative=True)

		print('Velocity North and East')
		send_global_velocity(uav, NORTH,EAST,0,DURATION)
		send_global_velocity(uav, 0,0,0,1)

		print('Yaw 90 relative (to previous yaw heading)')
		condition_yaw(uav,90,relative=True)

		print('Velocity South and East')
		send_global_velocity(uav, SOUTH,EAST,0,DURATION)
		send_global_velocity(uav, 0,0,0,1)




















	if False:
		'''
		The code in this conditional block shows custom examples of some of the functions available to the vehicle. 
		'''

		# Constants for testing purposes
		dNorth = 10 # [m]
		dEast = 10 # [m]
		uav.groundspeed = 5 # [m/s]

		dNorthHome = 0 # [m]
		dEastHome = 0 # [m]
		dDownHome = -10 # [m]
		DURATION_1 = 15 # [s]

		velocity_north = 1 # [m/s]
		velocity_east = 2 # [m/s]
		velocity_down = 0 # [m/s]
		DURATION_2 = 10 # [s]

		#------------------------------------
		# Position Navigation Command Tests
		#------------------------------------

		'''
		The goto command with no function parameter uses a relative position command. i.e. it travels dNorth and dEast relative to its current position. Negative dNorth is a southward movement, and negative dEast is a westward movement. 
		'''
		print '\n\nsimple_goto: Move %d m North and %d m East relative to current position\n' %(dNorth, dEast)
		goto(uav, 'UAV', dNorth, dEast)

		'''
		The goto command with a function parameter of 1 tells the goto command to use the goto_position_target_global_int function. It is effectively the same as goto with no function parameter (i.e. goto_position_target_global_int is effectively the same as simple_goto)
		'''
		print '\n\ngoto_position_target_global_int: Move %d m North and %d m East relative to current position\n' %(dNorth, dEast)
		goto(uav,'UAV',dNorth, dEast, 1)

		'''
		The goto_position_target_local_ned command uses the vehicle's home position to determine the target location. The target location is dNorth meters North of and dEast meters East of the home position.
		'''
		print '\n\ngoto_position_target_local_ned: Move %d m North, %d m East, and %d m Down relative to home location\n' %(dNorthHome, dEastHome, dDownHome)
		goto_position_target_local_ned(uav,dNorthHome,dEastHome,dDownHome)
		time.sleep(DURATION_1)

		#------------------------------------
		# Velocity Navigation Command Tests
		#------------------------------------

		# NOTE: Velocity commands must be sent every second. After 3 seconds with no velocity command, the uav will stop moving and hover in place.

		'''
		The send_ned_velocity command sets a speed in the NED frame for a specified DURATION. Internally, it uses the SET_POSITION_TARGET_LOCAL_NED MAVLink command with the velocity parameters enabled. The command then sleeps the thread for a time (DURATION), which defines the distance that will be travelled.
		'''
		send_ned_velocity(uav, velocity_north, velocity_east, velocity_down, DURATION_2)

		'''
		The send_global_velocity command sets a speed in the NED frame for a specified DURATION. Internally, it uses the SET_POSITION_TARGET_GLOBAL_INT MAVLink command with the velocity parameters enabled. The command then sleeps the thread for a time (DURATION), which defines the distance that will be travelled.
		'''
		send_global_velocity(uav, -velocity_north, -velocity_east, -velocity_down, DURATION_2)

		#------------------------------------
		# Condition Command Tests
		#------------------------------------

		'''
		From SITL testing, the following behavior is expected:
			- condition commands will not work until a nav command is sent
			- condition commands will work as long as at least one nav command is sent
			- a new nav command is not required to enact a new condition command
			- a goto command will overwrite a condition command, but a send velocity command will not
		'''

		print 'Yawing 90 degrees (relative) in increments of 30 degrees'
		for i in range(0,3):
			condition_yaw(uav, 30, relative = True)
			time.sleep(3)

		print 'simple_goto 20 m North and 20 m East from current position'
		goto(uav,20,20)

		print 'Setting ROI to home location'
		set_roi(uav, get_home_location(uav))

		print 'Setting ROI to current location'
		set_roi(uav, uav.location.global_relative_frame)

		print 'send_ned_velocity N for 15 seconds'
		send_ned_velocity(uav,2,0,0,15)

		print 'simple_goto 20 m South and 20 m West from current position'
		goto(uav,-20,20)
