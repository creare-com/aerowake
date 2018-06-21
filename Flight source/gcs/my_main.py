#!/usr/bin/env python2

# Required for importing mission and helper functions
import sys
sys.path.append('../')

# Required for messaging the drone
from dronekit import connect
from helper_functions import arm_vehicle, disarm_vehicle
import mission_rot

# Required for logging
import logging
import os.path
import time

#-------------------------------------------------------------------------------
#
# Global Parameters and Constants
#
#-------------------------------------------------------------------------------

# Set connection path to GCS
if len(sys.argv) > 1 and sys.argv[1] == 'sim':
	gcs_connect_path = '127.0.0.1:14556'
	gcs_baud = 115200
else:
	gcs_connect_path = '/dev/ttyACM0'
	gcs_baud = 57600

# Determine number of waypoints
num_wp = mission_rot.num_wp[0]

# Define a string that will print to show allowed user input
str_allowed_input = '\n\nAllowed input:\n -\n  Kill UAV motors when input starts with minus sign\n listen\n  Tell UAV to start listening to commands\n arm\n  Command UAV to arm throttle\n disarm\n  Command UAV to disarm throttle\n takeoff\n  Command UAV to takeoff to 10 m\n Waypoint Number (0-%d)\n  Navigate to designated waypoint\n clear\n  Clear current waypoint\n land\n  Command UAV to land\n help\n  Show this list of allowed inputs\n quit\n  Terminate program\n\n' %(num_wp - 1)

listening_err_str = 'First use listen command to tell UAV to listen.\n'

#-------------------------------------------------------------------------------
#
# Start Main Process
#
#-------------------------------------------------------------------------------

if __name__ == '__main__':

	# Log Setup
	logger = logging.getLogger('gcs_logger')
	logger.setLevel(logging.DEBUG)
	# Create file handler that sends all logger messages (DEBUG and above) to file
	fh = logging.FileHandler('%s/logs/aerowake-logs/gcs-%s.log' %(os.path.expanduser('~'),time.strftime('%m-%d-%Hh-%Mm-%Ss', time.localtime())))
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

	# GCS connection
	logger.info('Waiting for GCS')
	while True:
		try:
			gcs = connect(gcs_connect_path, baud = gcs_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except OSError:
			logger.critical('Cannot find device, is the gcs plugged in? Retrying...')
			time.sleep(5)
		except APIException:
			logger.critical('GCS connection timed out. Retrying...')

	logger.info('GCS pixhawk connected to GCS')

	logger.info('------------------SYSTEM IS READY!!------------------')
	logger.info('-----------------------------------------------------\n')

	# Callbacks for logging purposes

	@gcs.on_message('SYSTEM_TIME')
	def time_callback(self,attr_name, msg):
		logger.debug('gcsGPSTIME,%s' %msg)

	@gcs.on_message('LOCAL_POSITION_NED')
	def local_position_NED_callback(self,attr_name, msg):
		logger.debug('localPosNED,%s' %msg)

	#-----------------------------------------------------------------------------
	#
	# Control Code
	#
	#-----------------------------------------------------------------------------

	arm_vehicle(gcs,'GCS')
	print str_allowed_input

	# Set initial GCS value. For safety, the GCS should start and end on this value. This value tells the UAV to follow the previous command. If no previous command exists, then this value tells the UAV to do nothing. 
	gcs.parameters['PIVOT_TURN_ANGLE'] = 100

	'''
	This while loop waits for user input, and then commands the UAV to perform some action. The command is sent by setting a parameter on the GCS. The UAV is constantly reading this parameter and acting according to its value. The parameter PIVOT_TURN_ANGLE accepts values from 0 - 359, inclusive, and has no effect on GCS performance. 
	
	The parameter values corresponding actions to be performed are:
	 	100		UAV will stop listening to these commands
	 					UAV will follow prev command, or do nothing if no command sent yet
	 	101		UAV will begin listening to these commands
		102		UAV will clear its current waypoint
		103		UAV kills its motors immediately
	 	359		UAV will arm
	 	358		UAV will disarm
	 	357		UAV will takeoff to a pre-programmed height and relative position
	 	356		UAV will land according to its landing protocol
	 	0+		UAV will navigate to the waypoint at the index specified
	 					The acceptable waypoint indices are 0 through num_wp - 1
	
	If an invalid input is entered, such as a typo or a waypoint number that does not exist, the UAV will follow the previous command. The GCS will echo this behavior to the terminal, notifying the user of the UAV's behavior in the event of an invalid input. 
	
	When the quit command is given, the UAV will continue to follow its most recent command. 
	'''

	user_in = None
	uav_listening = False
	prev_command = 'No previous command.\n'
	try:
		while not user_in == 'quit':
			# Wait for user input
			print 'Enter command:'
			user_in = raw_input()
			logger.debug('rawUserInput,%s',user_in)

			# Adjust user input for conditional statements
			user_in = user_in.lower()

			# Perform some action based upon the user input
			invalid_input = True
			if len(user_in) > 0 and user_in[0] == '-':
				invalid_input = False
				gcs.parameters['PIVOT_TURN_ANGLE'] = 103
				logger.info('Killing UAV motors.\n')

			elif user_in == 'help':
				invalid_input = False
				logger.info(str_allowed_input)

			elif user_in == 'quit':
				invalid_input = False
				# Will terminate on next loop. UAV will continue with previous command
				logger.info('Terminating GCS loop.')
				logger.info(' UAV is following previous command of: \n %s' %(prev_command))

			elif user_in == 'listen':
				invalid_input = False
				# UAV knows that 101 means start listening to commands
				gcs.parameters['PIVOT_TURN_ANGLE'] = 101
				logger.info('Commanding UAV to listen to commands\n')
				prev_command = 'Command UAV to listen to commands\n'
				uav_listening = True

			elif not uav_listening:
				# UAV not listening and input is something other than 'listen'
				invalid_input = False
				logger.info(listening_err_str)
			else:
				# UAV is listening
				if user_in == 'arm':
					invalid_input = False
					# UAV knows that 359 means arm
					gcs.parameters['PIVOT_TURN_ANGLE'] = 359
					logger.info('Commanding UAV to Arm\n')
					prev_command = 'Command UAV to Arm\n'

				elif user_in == 'disarm':
					invalid_input = False
					# UAV knows that 358 means disarm
					gcs.parameters['PIVOT_TURN_ANGLE'] = 358
					logger.info('Commanding UAV to Disarm\n')
					prev_command = 'Command UAV to Disarm\n'

				elif user_in == 'takeoff':
					invalid_input = False
					# UAV knows that 357 means takeoff
					gcs.parameters['PIVOT_TURN_ANGLE'] = 357
					logger.info('Commanding UAV to Takeoff\n')
					prev_command = 'Command UAV to Takeoff\n'

				elif user_in == 'land':
					invalid_input = False
					# UAV knows that 356 means land
					gcs.parameters['PIVOT_TURN_ANGLE'] = 356
					logger.info('Commanding UAV to Land\n')
					prev_command = 'Command UAV to Land\n'

				elif user_in == 'clear':
					invalid_input = False
					# UAV knows that 102 means clear
					gcs.parameters['PIVOT_TURN_ANGLE'] = 102
					logger.info('Commanding UAV to clear the current waypoint\n')
					prev_command = 'Command UAV to clear the current waypoint\n'

				else:
					try:
						user_in = int(user_in)
						# If user enters an integer, set the chosen GCS parameter to that integer value. The UAV will read this parameter and navigate to that waypoint. E.g. if user enters 2, then UAV will navigate to the waypoint at index 2. 
						if user_in >= 0 and user_in < num_wp:
							invalid_input = False
							if uav_listening:
								logger.info('Commanding UAV to waypoint %s\n' %(user_in))
								prev_command = 'Command UAV to waypoint %s\n' %(user_in)
								gcs.parameters['PIVOT_TURN_ANGLE'] = user_in
							else:
								print listening_err_str
					except ValueError:
						# Will be invalid input and will print such as defined below
						pass

			if invalid_input:
				logger.info('Invalid input. Type \'help\' for allowed commands.')
				logger.info(' UAV is following previous command of: \n %s' %(prev_command))
				# print str_allowed_input
	except KeyboardInterrupt:
		logger.info('\nGot CTRL+C. Cleaning up and exiting.\n')

	#------------------------------------
	# Outside of while loop
	#------------------------------------

	# Set final GCS value. For safety, the GCS should start and end on this value. This value tells the UAV to follow the previous command. If no previous command exists, then this value tells the UAV to do nothing. 
	gcs.parameters['PIVOT_TURN_ANGLE'] = 100

	# Disarm and close GCS object before exiting script
	disarm_vehicle(gcs,'GCS')
	gcs.close()

	print('GCS program completed\n')
