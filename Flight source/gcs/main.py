#!/usr/bin/env python2

# Required for importing mission and helper functions
import sys
sys.path.append('../')

# Required for messaging pixhawks
from dronekit import connect
from helper_functions import arm_vehicle, disarm_vehicle
import base_mission

# Required for logging
import logging
import os.path
import time

# Required for reel control
from multiprocessing import Queue
from Queue import Empty
from reel.reel import reel_run
import math

#-------------------------------------------------------------------------------
#
# Global Parameters and Constants
#
#-------------------------------------------------------------------------------

# Set if you are using the reel and tether length safety factor
using_reel = True
safety_factor = 1.15

# Set rotate command limits
max_deg = 20

# Set filename
filename = sys.argv[1]

# Set connection path to GCS
if len(sys.argv) > 2 and sys.argv[2] == 'sim':
	gcs_connect_path = '127.0.0.1:14556'
	gcs_baud = 115200
else:
	gcs_connect_path = '/dev/ttyAMA0' # For Creare RPi
	gcs_baud = 115200 # For Creare RPi
	#gcs_connect_path = '/dev/ttyACM0' # For laptop USB
	#gcs_baud = 57600 # For laptop USB

# Determine number of waypoints and takeoff altitude
num_wp = base_mission.num_wp
alt_takeoff = base_mission.alt_takeoff
gcs_mission = [base_mission.wp_N, base_mission.wp_E, base_mission.wp_D]

# Define help strings
str_allowed_input = '''

Allowed input:
 listen
	Tell UAV to start listening to commands
 d
	Report tether data
 r
	Reel tether in to landing length
 c
	Clear a faulted motor controller and renable the reel
 #
  Set tether to 10x the specified length [m]
 h
 	Show this list
 q
	End GCS\'s main.py

'''

# Define parameters to use
cmd_param = 'PIVOT_TURN_ANGLE'
ack_param = 'ACRO_TURN_RATE'
bearing_param = 'RNGFND_TURN_ANGL'

# Reel function
def get_reel_data():
	'''
	Returns the current length of the tether in meters as believed by the reel controller.
	
	Expected to return {"L": <length in meters as double>, "T": <tension in newtons as double>}
	'''
	try:
		reel_reading = data_from_reel.get(False)
	except Empty:
		reel_reading = {'L':'-', 'T':'-'}
	logger.debug('reelData,%s',reel_reading)
	return reel_reading

#-------------------------------------------------------------------------------
#
# Start Main Process
#
#-------------------------------------------------------------------------------

if __name__ == '__main__':

	# Log Setup
	logger = logging.getLogger('pixhawk_logger')
	logger.setLevel(logging.DEBUG)
	# Create file handler that sends all logger messages (DEBUG and above) to file
	logfile = '%s/logs/gcs-logs/%s-gcs-%s.log' %(os.path.expanduser('~'),filename,time.strftime('%Y-%m-%d-%Hh-%Mm-%Ss', time.localtime()))
	fh = logging.FileHandler(logfile)
	print 'Logging GCS data to %s' %(logfile)
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

	if using_reel:
		# Start reel controller
		commands_to_reel = Queue()
		data_from_reel = Queue()
		try:
			reel = reel_run(commands_to_reel, data_from_reel, filename)
		except Exception as e:
			logging.critical('Problem connecting to reel. Aborting.')
			raise e
		reel.start()
		commands_to_reel.put({"cmd":"goto", "L":0})
	else:
		logger.info('NOT USING REEL')

	# GCS connection
	logger.info('Waiting for GCS\n')
	while True:
		try:
			gcs = connect(gcs_connect_path, baud = gcs_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except Exception as e:
			logger.critical('GCS failed to connect with message: %s' %(e.message))
			raise e

	logger.info('GCS pixhawk connected to GCS\n')

	# Arm the GCS so that dataflash logging occurs
	arm_vehicle(gcs,'GCS')

	#------------------------------------
	# Listeners for Logging
	#------------------------------------

	@gcs.on_message('SYSTEM_TIME')
	def time_callback(self, attr_name, msg):
		logger.debug('gcsGPSTIME, %s' %msg)

	@gcs.on_message('LOCAL_POSITION_NED')
	def local_position_NED_callback(self,attr_name, msg):
		logger.debug('localPosNED, %s' %msg)

	@gcs.on_message('ATTITUDE')
	def attitude_callback(self, attr_name, msg):
		logger.debug('gcsATT, %s' %msg)

	# ack_param is used as an acknowledge parameter by the UAV.
	@gcs.parameters.on_attribute(ack_param)
	def UAV_parameter_callback(self, attr_name, UAV_param):
		if UAV_param == 102:
			# clear current waypoint
			logger.info('UAV received command to clear current waypoint.\n')
		elif UAV_param == 101:
			# begin listening
			logger.info('UAV received command to listen.\n')
		elif UAV_param == 100:
			# stop listening
			logger.info('UAV received command to stop listening.\n')
		elif UAV_param == 359:
			# arm
			logger.info('UAV received command to arm.\n')
		elif UAV_param == 358:
			# disarm
			logger.info('UAV received command to disarm.\n')
		elif UAV_param == 357:
			# takeoff
			logger.info('UAV received command to takeoff.\n')
		elif UAV_param == 356:
			# land
			logger.info('UAV receieved command to land.\n')
		elif UAV_param == 355:
			#rotate
			logger.info('UAV received command to rotate.\n')
		elif UAV_param == 200:
			# increase altitude by 1 meter
			logger.info('UAV received command to increase altitude.\n')
		elif UAV_param == 201:
			# decrease altitude by 1 meter
			logger.info('UAV received command to decrease altitude.\n')
		elif UAV_param == 202:
			# increase wpnav_speed by 0.5 m/s
			logger.info('UAV received command to increase wpnav_speed.\n')
		elif UAV_param == 203:
			# decrease wpnav_speed by 0.5 m/s
			logger.info('UAV received command to decrease wpnav_speed.\n')
		elif UAV_param == 204:
			# increase wpnav_accel by 0.3 m/s
			logger.info('UAV received command to increase wpnav_accel.\n')
		elif UAV_param == 205:
			# decrease wpnav_accel by 0.3 m/s
			logger.info('UAV received command to decrease wpnav_accel.\n')
		elif UAV_param >= 0 and UAV_param < num_wp:
			# commanding waypoint
			logger.info('UAV received command to go to waypoint %d.\n', UAV_param)
		else:
			# something went wrong
			logger.warning('WARNING: UAV acknowledged an unknown command.\n')

	# @gcs.on_message('*')
	# def any_message_listener(self, name, message):
	# 	# Comment out this listener before flight or file created will be enormous.
	# 	logger.info('fromGCS: %s :: %s',name,message)

	logger.info('------------------SYSTEM IS READY!!------------------')
	logger.info('-----------------------------------------------------\n')



	#-----------------------------------------------------------------------------
	#
	# Control Code
	#
	#-----------------------------------------------------------------------------

	# Print the allowed commands
	print str_allowed_input

	# Set initial GCS value. For safety, the GCS should start and end on this value. This value tells the UAV to follow the previous command. If no previous command exists, then this value tells the UAV to do nothing.
	gcs.parameters[cmd_param] = 100

	# Set initial relative bearing to 0
	bearing = 0
	gcs.parameters[bearing_param] = bearing

	'''
	This while loop waits for user input, and then commands the UAV to perform some action. The command is sent by setting a parameter on the GCS. The UAV is constantly reading this parameter and acting according to its value. The parameter PIVOT_TURN_ANGLE accepts values from 0 - 359, inclusive, and has no effect on GCS performance.

	The parameter values corresponding actions to be performed are:
		100     UAV will stop listening to these commands
						UAV will either follow prev command, or do nothing if no command sent yet
		101     UAV will begin listening to these commands
		102     UAV will clear its current waypoint
		200     UAV will increase its mission altitude by 1 meter
		201     UAV will decrease its mission altitude by 1 meter
		202     UAV will increase its WPNAV_SPEED by 0.5 m/s
		203     UAV will decrease its WPNAV_SPEED by 0.5 m/s
		204     UAV will increase its WPNAV_ACCEL altitude by 0.3 m/s/s
		205     UAV will decrease its WPNAV_ACCEL altitude by 0.3 m/s/s
		359     UAV will arm
		358     UAV will disarm
		357     UAV will takeoff to a pre-programmed height
		356     UAV will land according to its landing protocol
		355	UAV will rotate to bearing given by parameter RNGFND_TURN_ANGL
		300	Reserved for resetting ack_param for repeated commands
		0+      UAV will navigate to the waypoint at the index specified
						The acceptable waypoint indices are 0 through num_wp - 1

	If an invalid input is entered, such as a typo or a waypoint number that does not exist, the UAV will follow the previous command. The GCS will echo this behavior to the terminal, notifying the user of the UAV's behavior in the event of an invalid input.

	When the quit command is given, the UAV will continue to follow its most recent command.
	'''

	user_in = None
	prev_command = 'No previous command.'

	try:
		while not user_in == 'q':
			# Wait for user input
			print 'Enter command:'
			user_in = raw_input()
			logger.debug('rawUserInput,%s',user_in)

			# Adjust user input for conditional statements
			user_in = user_in.lower()

			# Perform some action based upon the user input
			invalid_input = True

			if user_in == 'h':
				invalid_input = False
				logger.info(str_allowed_input)
				continue

			elif user_in == 'q':
				invalid_input = False
				# Will terminate on next loop. UAV will continue with previous command
				logger.info('Terminating GCS loop.')
				continue

			elif user_in == 'listen':
				invalid_input = False
				# UAV knows that 101 means start listening to commands
				gcs.parameters[cmd_param] = 101
				logger.info('Commanding UAV to listen to commands')

			elif user_in == 'd':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to report data.')
					reel_data = get_reel_data()
					logger.info(' Reel at: %s',reel_data)
				else:
					logger.info('rgetdata called when not using reel.')
			
			elif user_in == 'r':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to reel in to landing.')
					commands_to_reel.put({"cmd":"goto", "L":0.2})
				else:
					logger.info('reelin called when not using reel.')

			elif user_in == 'clearfault':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to clear fault.')
					commands_to_reel.put({"cmd":"clearfault"})
				else:
					logger.info('clearfault called when not using reel.')

			elif user_in == 'c':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to clear fault and enable.')
					commands_to_reel.put({"cmd":"clearfaultandenable"})
				else:   
					logger.info('clearfaultandenable called when not using reel.')

			elif user_in == 'geterror':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to report errors.')
					commands_to_reel.put({"cmd":"geterror"})
				else:
					logger.info('geterror called when not using reel.')

			else:
				try:
					user_in = float(user_in)
					if user_in >= 0:
						invalid_input = False
						# Command reel to 10x length specified by user_in
						if using_reel:
							dist = user_in*10.0
							logger.info('Commanding reel to %s meters', dist)
							commands_to_reel.put({"cmd":"goto", "L":dist})
						else:
							logger.info('Cannot set tether length when not using reel.')

				except ValueError:
					# Will be invalid input and will print such as defined below
					pass

			if invalid_input:
				logger.info('Invalid input. Type \'help\' for allowed commands.')

			print('')

	except KeyboardInterrupt:
		logger.info('\nGot CTRL+C. Cleaning up and exiting.\n')

	#------------------------------------
	# Outside of while loop
	#------------------------------------

	# Set final GCS value. For safety, the GCS should start and end on this value. This value tells the UAV to follow the previous command. If no previous command exists, then this value tells the UAV to do nothing.
	gcs.parameters[cmd_param] = 100

	# Close GCS object before exiting script
	disarm_vehicle(gcs,'GCS')
	gcs.close()

	if using_reel:
		# Shutdown reel controller
		commands_to_reel.put({'cmd':'exit'})
		reel.join()

	logger.info('GCS program completed\n')
