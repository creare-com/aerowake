#!/usr/bin/env python2

# Import necessary items from parent directory, then undo changes to path
# Required for controlling the UAV
from inspect import getsourcefile
import os.path as path, sys
current_dir = path.dirname(path.abspath(getsourcefile(lambda:0)))
sys.path.insert(0, current_dir[:current_dir.rfind(path.sep)])
from helper_functions import arm_vehicle, condition_yaw, disarm_vehicle, emergency_stop, get_bearing, get_distance_metres, get_home_location, get_location_metres, goto, goto_position_target_local_ned, goto_reference, land, send_global_velocity, send_ned_velocity, set_roi, takeoff
from rotate_mission import rotate
import base_mission
sys.path.pop(0)

# Required for the vision-based yaw system
import rospy
from std_msgs.msg import Int16

# Required for controlling the UAV
import logging
import numpy as np
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

# Required for logging
import os.path

#-------------------------------------------------------------------------------
#
# Define ROS Node for Flight
#
#-------------------------------------------------------------------------------

class DroneCommanderNode(object):
	'''
	This class will communicate with the Pixhawks on the GCS and UAV. It will then generate MAVLink commands to send to the UAV Pixhawk that will control the UAV.

	Most commands are generated based upon what the GCS tells the UAV to do. The UAV reads a GCS parameter, the value of which corresponds to a particular action.

	The UAV also obtains commands from the /yaw_deg ROS topic. This topic contains an integer (Int16) value in degrees corresponding to the desired yaw of the UAV. A positive value indicates CCW yaw of the specified magnitude, while a negative value indicates CW yaw.
	'''

	def __init__(self,uav_handle,gcs_handle,logger_name):
		# Initialize private variables
		self.__yaw_cmd = 0 # [deg]

		# Initialize other variables
		uav = uav_handle
		gcs = gcs_handle
		logger = logging.getLogger(logger_name)

		# Subscribe to topic that reports yaw commands
		self.sub_yaw_deg = rospy.Subscriber("yaw_deg",Int16,self.cbYawDeg)

		# Extract mission and define a local version called uav_mission
		orig_mission = [base_mission.wp_N, base_mission.wp_E, base_mission.wp_D]
		num_wp = base_mission.num_wp
		alt_takeoff = base_mission.alt_takeoff

		# Define parameters to use
		cmd_param = 'PIVOT_TURN_ANGLE'
		ack_param = 'ACRO_TURN_RATE'
		bearing_param = 'RNGFND_TURN_ANGL'

		#---------------------------------------------------------------------------
		#
		# Control Code
		#
		#---------------------------------------------------------------------------

		#------------------------------------
		# Control
		#------------------------------------

		'''
		This while loop performs the following functions:
			- read GCS position
			- read GCS parameter value of PIVOT_TURN_ANGLE to determine next UAV action
			- perform an action based upon the GCS parameter value

		The parameter values corresponding actions to be performed are:
			100     UAV will stop listening to these commands
							UAV will either follow prev command, or do nothing if no command sent yet
			101     UAV will begin listening to these commands
			102     UAV will clear its current waypoint
			103			UAV will end its main.py if it is disarmed
			359     UAV will arm
			358     UAV will disarm
			357     UAV will takeoff to a pre-programmed height
			356     UAV will land according to its landing protocol
			355		  UAV will rotate to bearing given by parameter RNGFND_TURN_ANGL
			0+      UAV will navigate to the waypoint at the index specified
							The acceptable waypoint indices are 0 through num_wp - 1
		'''

		# Ensure that at startup the UAV will not be tracking any waypoint
		current_wp = None

		# Ensure that at startup the UAV will not be listening
		gcs.parameters[cmd_param] = 100

		# We start with a 0 rotation on the mission to match what we have on the gcs
		gcs.parameters[bearing_param] = 0

		# Set UAV acceleration limit and groundspeed
		uav.parameters['WPNAV_ACCEL'] = 75 # 50-500 [cm/s/s]
		uav.parameters['WPNAV_SPEED'] = 250 # 20-2000 by 50 [cm/s]
		# uav.groundspeed = 5 # [m/s]

		logger.debug('WPNAV_ACCEL,%s' %(uav.parameters['WPNAV_ACCEL']))
		logger.debug('WPNAV_SPEED,%s' %(uav.parameters['WPNAV_SPEED']))

		command = 100 # This varibale is an echo for the variable param in order to disallow repeated commands
		i = 0
		in_the_air = False
		listening = False
		continue_loop = True
		while continue_loop and not rospy.is_shutdown():
			param = gcs.parameters[cmd_param]
			gcs.parameters[ack_param] = param # Acknowledge
			gcsLoc = gcs.location.global_frame
			uavLoc = uav.location.global_frame
			logger.debug('gcsLoc,%s', gcsLoc)
			logger.debug('uavLoc,%s', uavLoc)

			'''
			The variable 'command' is what controls the drone. It is only updated when a valid and non-repeated param value is set. This ensures that the UAV does not continue commanding the same thing over and over.

			The 'command' variable can only be set while 'listening' is True. If the UAV is not listening to the GCS, then it will follow the most recent 'command' variable.

			It is occasionally desireable for waypoint commands to be repeated, so waypoint commands are handled slightly differently.

			The parameter for 'stop listening' will be immediately passed through since repeated 'stop listening' commands are also occasionally desired.
			'''

			if listening:
				logger.info('Listening')
				if param == 100:
					logger.info('Got stop listening command')
					listening = False
				else:
					command = param
					if param == 102 and not command == param:
						logger.info('Got clear waypoint command')
					elif param == 103 and not command == param:
						logger.info('Got end UAV main.py command')
					elif param == 359 and not command == param:
						logger.info('Got arm command')
					elif param == 358 and not command == param:
						logger.info('Got disarm command')
					elif param == 357 and not command == param:
						logger.info('Got takeoff command')
					elif param == 356 and not command == param:
						logger.info('Got land command')
					elif param == 355 and not command == param:
						logger.info('Got rotate command')
					elif param >= 0 and param < num_wp:
						# NOTE: GCS will not allow a non-existent index to be passed. The handling of negative indices is included in the conditional above for comprehension.
						logger.info('Got navigate to waypoint %d command',param)
						current_wp = int(param)
			else:
				logger.info('Not listening')
				if param == 101:
					logger.info('Got start listening command')
					listening = True

			# Do the action that corresponds to the current value of 'command'
			if command == 100:
				logger.info('Waiting for command')
			elif command == 102 and current_wp is not None:
				logger.info('Clearing current waypoint')
				current_wp = None
			elif not in_the_air:
				if command == 359 and not uav.armed:
					logger.info('Arming')
					arm_vehicle(uav,'UAV')
					# Set arming altitude as the desired reference altitude
					alt_initial = uav.location.global_frame.alt
					# Yaw is reported from -pi to pi with zero pointing North, so we need to convert to 0 to 360 and keep zero pointing North
					bearing = uav.attitude.yaw*180/np.pi - 180
					if bearing < 0:
						bearing = 360 + bearing
					logger.info('Rotating mission by %s degrees at arming' %(bearing))
					uav_mission = rotate(orig_mission,bearing)
				elif command == 358 and uav.armed:
					logger.info('Disarming')
					disarm_vehicle(uav,'UAV')
				elif command == 357 and uav.armed:
					logger.info('Taking off')
					takeoff(uav,'UAV',alt_takeoff)
					goto_reference(uav, uav.location.global_frame, 0, 0, 0)
					condition_yaw(uav, 0, relative = True)
					in_the_air = True
					current_wp = None
				elif command == 103 and not uav.armed:
					continue_loop = False
			elif in_the_air: # Explicit for comprehension
				if command == 356:
					logger.info('Landing')
					land(uav,'UAV')
					in_the_air = False
				if command == 355 and not current_wp is None:
					# Only allow rotation of mission if already following a waypoint
					bearing = gcs.parameters[bearing_param]
					logger.info('Rotating mission by %s degrees after GCS operator command' %(bearing))
					uav_mission = rotate(uav_mission,bearing)
					# Tell the UAV to continue following the current waypoint through the standard pipeline
					gcs.parameters[cmd_param] = current_wp
					# Reset the bearing parameter to zero to prevent continual rotation
					gcs.parameters[bearing_param] = 0
				else:
					if not current_wp is None:
						logger.info('Tracking waypoint %d',current_wp)
						refLoc = gcs.location.global_frame
						refLoc.alt = alt_initial
						logger.debug('refLocNav,%s',refLoc)
						dNorth = uav_mission[0][current_wp]
						dEast = uav_mission[1][current_wp]
						dDown = uav_mission[2][current_wp]
						# Calculate desired location for logging purposes only. Actual desired location is calculated within the goto_reference function.
						desLoc = get_location_metres(refLoc, dNorth, dEast)
						desLoc.alt = alt_initial - dDown
						logger.debug('desLoc,%s',desLoc)
						logger.debug('currentMissionNED,%s,%s,%s',dNorth,dEast,dDown)
						goto_reference(uav, refLoc, dNorth, dEast, dDown)
						# Only condition yaw once every so many seconds
						if np.mod(i,3) == 0 and get_distance_metres(uav.location.global_frame,refLoc) > 3:
							yaw_rel = self.__yaw_cmd
							condition_yaw(uav, yaw_rel, relative = True)
							logger.debug('RELYAW,%s',yaw_rel)
						i = i + 1
					else:
						logger.info('In the air, but not tracking a waypoint')

			print ''
			gcs.parameters[ack_param] = param # 2nd acknowledge since the first may not have worked
			time.sleep(0.5)

		#------------------------------------
		# Terminating
		#------------------------------------

		if rospy.is_shutdown():
			logger.info('Terminating program since ROS is shutdown. Not changing UAV status.\n')
		else:
			# The example is completing. LAND at current location.
			land(uav,'UAV')

			# Disarm and close UAV object before exiting script
			disarm_vehicle(uav,'UAV')

		uav.close()

		logger.info('UAV program completed.\n')

	def cbYawDeg(self, data):
		self.__yaw_cmd = data.data

if __name__ == '__main__':

	#-----------------------------------------------------------------------------
	#
	# Global Parameters and Constants
	#
	#-----------------------------------------------------------------------------

	# Set connection path to UAV and GCS
	# uav_connect_path = '127.0.0.1:14552'
	# uav_baud = 115200
	# gcs_connect_path = '127.0.0.1:14554'
	# gcs_baud = 115200

	# uav_connect_path = '/dev/ttyACM0' # Use for odroid through pixhawk usb cord
	# uav_connect_path = '/dev/ttyUSB0' # Use for odroid through usb to serial converter
	# uav_connect_path = '/dev/ttySAC0' # Use for odroid through GPIO pins
	uav_connect_path = '/dev/pixhawk' # Use after configuring symbolic link through udevadm
	uav_baud = 57600

	# gcs_connect_path = '/dev/ttyUSB0' # Use for telemetry radio through usb port
	gcs_connect_path = '/dev/radioacl33' # Use after configuring symbolic link through udevadm
	gcs_baud = 57600

	#-----------------------------------------------------------------------------
	#
	# Start ROS Node
	#
	#-----------------------------------------------------------------------------

	# Log Setup
	logger_name = 'uav_logger'
	logger = logging.getLogger(logger_name)
	logger.setLevel(logging.DEBUG)
	# Create file handler that sends all logger messages (DEBUG and above) to file
	logfile = '%s/logs/uav-logs/uav-%s.log' %(os.path.expanduser('~'),time.strftime('%Y-%m-%d-%Hh-%Mm-%Ss', time.localtime()))
	fh = logging.FileHandler(logfile)
	print "Logging to %s" %(logfile)
	fh.setLevel(logging.DEBUG)
	# Create console handler that sends some messages (INFO and above) to screen
	ch = logging.StreamHandler(sys.stdout)
	ch.setLevel(logging.INFO)
	# Set the log format for each handler
	form_fh = logging.Formatter('%(created)s,%(relativeCreated)s,%(funcName)s,%(levelname)s: %(message)s')
	form_ch = logging.Formatter('%(levelname)s: %(message)s')
	fh.setFormatter(form_fh)
	ch.setFormatter(form_ch)
	# Add the handler to the logger
	logger.addHandler(fh)
	logger.addHandler(ch)

	# UAV connection
	logger.info('Waiting for UAV')
	while True:
		try:
			uav = connect(uav_connect_path, baud = uav_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except Exception as e:
			logger.critical('UAV failed to connect with message: %s' %(e.message))
			raise e

	logger.info('UAV pixhawk connected to UAV')

	# GCS connection
	logger.info('Waiting for GCS')
	while True:
		try:
			gcs = connect(gcs_connect_path, baud = gcs_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except Exception as e:
			logger.critical('GCS failed to connect with message: %s' %(e.message))
			raise e

	logger.info('GCS pixhawk connected to UAV')

	#------------------------------------
	# Listeners for Logging
	#------------------------------------

	@gcs.on_message('SYSTEM_TIME')
	def gcs_time_callback(self,attr_name, msg):
		logger.debug('gcsGPSTIME,%s' %msg)

	timed_out_gcs = False
	@gcs.on_attribute('last_heartbeat')
	def gcs_last_heartbeat_listener(self, attr_name, value):
		if(attr_name is 'last_heartbeat'):
			global timed_out_gcs
			if value > 3 and not timed_out_gcs:
				timed_out_gcs = True
				logger.critical('GCS pixhawk connection lost!')
			if value < 3 and timed_out_gcs:
				timed_out_gcs = False;
				logger.info('GCS pixhawk connection restored.')

	gps_lock_odroid_time = 0
	gps_lock_gps_time = 0
	@uav.on_message('SYSTEM_TIME')
	def uav_time_callback(self, attr_name, msg):
		global gps_lock_gps_time
		if(gps_lock_gps_time is 0 and uav.gps_0.fix_type == 3):
			gps_lock_gps_time = msg.time_unix_usec/1000000
			logger.info('UAV got GPS lock at GPS time of: %s', gps_lock_gps_time)
			gps_lock_odroid_time = '%0.4f' % time.time()
			logger.info('UAV got GPS lock at ODROID time of: %s', gps_lock_odroid_time)

	timed_out_uav = False
	@uav.on_attribute('last_heartbeat')
	def uav_last_heartbeat_listener(self, attr_name, value):
		if(attr_name is 'last_heartbeat'):
			global timed_out_uav
			if value > 3 and not timed_out_uav:
				timed_out_uav = True
				logger.critical('UAV pixhawk connection lost!')
			if value < 3 and timed_out_uav:
				timed_out_uav = False;
				logger.info('UAV pixhawk connection restored.')

	@uav.on_attribute('armed')
	def arm_disarm_callback(self,attr_name, msg):
		logger.info('UAV is now %sarmed ' % ('' if uav.armed else 'dis'))

	@uav.on_attribute('mode')
	def mode_callback(self,attr_name, mode):
		logger.info('UAV mode changed to %s' % mode.name)

	@uav.on_message('NAV_CONTROLLER_OUTPUT')
	def nav_callback(self,attr_name, msg):
		logger.debug('NAVCTRLOUT,%s' %msg)

	@uav.on_message('SYSTEM_TIME')
	def uav_time_callback(self,attr_name, msg):
		logger.debug('uavGPSTIME,%s' %msg)

	@uav.on_message('ATTITUDE')
	def attitude_callback(self, attr_name, msg):
		logger.debug('uavATT,%s' %msg)

	@uav.on_message('LOCAL_POSITION_NED')
	def local_position_NED_callback(self,attr_name, msg):
		logger.debug('localPosNED,%s' %msg)

	# @uav.on_message('*')
	# def any_message_listener(self, name, message):
	# 	# Comment out this listener before flight or file created will be enormous.
	# 	logger.info('fromUAV: %s :: %s',name,message)

	logger.info('------------------SYSTEM IS READY!!------------------')
	logger.info('-----------------------------------------------------\n')

	# Initialize the node
	rospy.init_node('flight_companion_node')

	# Create the node
	node = DroneCommanderNode(uav,gcs,logger_name)

	# Spin
	rospy.spin()
