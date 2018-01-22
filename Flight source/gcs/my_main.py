#!/usr/bin/env python2

import sys
sys.path.append('../')

import logging
import mission
import time

from dronekit import connect
from helper_functions import arm_vehicle, disarm_vehicle

#-------------------------------------------------------------------------------
#
# Global Parameters and Constants
#
#-------------------------------------------------------------------------------

# Set connection path to GCS
gcs_connect_path = '127.0.0.1:14556'
gcs_baud = 115200

# Determine number of waypoints
num_wp = mission.num_wp

# Define a string that will print to show allowed user input
str_allowed_input = '\n\nAllowed input:\n listen\n  Tell UAV to start listening to commands\n arm\n  Command UAV to arm throttle\n disarm\n  Command UAV to disarm throttle\n Waypoint Number (0-%d)\n  Navigate to designated waypoint\n takeoff\n  Command UAV to takeoff to 10 m\n land\n  Command UAV to land\n help\n  Show this list of allowed inputs\n quit\n  Terminate program\n\n' %(num_wp - 1)

listening_err_str = 'First use listen command to tell UAV to listen.\n'

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

	# GCS connection
	logging.info('Waiting for GCS')
	while True:
		try:
			gcs = connect(gcs_connect_path, baud = gcs_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except OSError:
			logging.critical('Cannot find device, is the gcs plugged in? Retrying...')
			time.sleep(5)
		except APIException:
			logging.critical('GCS connection timed out. Retrying...')

	logging.info('GCS pixhawk connected to GCS')

	# Listeners
	@gcs.on_attribute('gimbal')
	def ground_speed_listener(self, attr_name, value):
		print 'gimbal: ', value

	logging.info('------------------SYSTEM IS READY!!------------------')
	logging.info('-----------------------------------------------------\n')

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
	
	The acceptable parameter values are:
	 	100		UAV will follow previous command, or do nothing if no command sent yet
	 	101		UAV will begin listening to these commands
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
	while not user_in == 'quit':
		# Wait for user input
		print 'Enter command:'
		user_in = raw_input()

		# Adjust user input for conditional statements
		user_in = user_in.lower()

		# Perform some action based upon the user input
		invalid_input = True
		if user_in.isalnum():

			if user_in == 'listen':
				# UAV knows that 101 means start listening to commands
				gcs.parameters['PIVOT_TURN_ANGLE'] = 101
				invalid_input = False
				print 'Commanding UAV to listen to commands\n'
				prev_command = 'Command UAV to listen to commands\n'
				uav_listening = True

			elif user_in == 'arm':
				invalid_input = False
				if uav_listening:
					# UAV knows that 359 means arm
					gcs.parameters['PIVOT_TURN_ANGLE'] = 359
					print 'Commanding UAV to Arm\n'
					prev_command = 'Command UAV to Arm\n'
				else:
					print listening_err_str

			elif user_in == 'disarm':
				invalid_input = False
				if uav_listening:
					# UAV knows that 358 means disarm
					gcs.parameters['PIVOT_TURN_ANGLE'] = 358
					print 'Commanding UAV to Disarm\n'
					prev_command = 'Command UAV to Disarm\n'
				else:
					print listening_err_str

			elif user_in == 'takeoff':
				invalid_input = False
				if uav_listening:
					# UAV knows that 357 means takeoff
					gcs.parameters['PIVOT_TURN_ANGLE'] = 357
					print 'Commanding UAV to Takeoff\n'
					prev_command = 'Command UAV to Takeoff\n'
				else:
					print listening_err_str

			elif user_in == 'land':
				invalid_input = False
				if uav_listening:
					# UAV knows that 356 means land
					gcs.parameters['PIVOT_TURN_ANGLE'] = 356
					print 'Commanding UAV to Land\n'
					prev_command = 'Command UAV to Land\n'
				else:
					print listening_err_str

			elif user_in == 'help':
				invalid_input = False
				print str_allowed_input

			elif user_in == 'quit':
				invalid_input = False
				# Will terminate on next loop. UAV will continue with previous command
				print 'Terminating GCS loop.'
				print ' UAV is following previous command of: \n %s' %(prev_command)
			else:
				try:
					user_in = int(user_in)
					# If user enters an integer, set the chosen GCS parameter to that integer value. The UAV will read this parameter and navigate to that waypoint. E.g. if user enters 2, then UAV will navigate to the waypoint at index 2. 
					if user_in >= 0 and user_in < num_wp:
						invalid_input = False
						if uav_listening:
							print 'Commanding UAV to waypoint %s\n' %(user_in)
							prev_command = 'Command UAV to waypoint %s\n' %(user_in)
							gcs.parameters['PIVOT_TURN_ANGLE'] = user_in
						else:
							print listening_err_str
				except ValueError:
					# Will be invalid input and will print such as defined below
					pass

		if invalid_input:
			print 'Invalid input.'
			print ' UAV is following previous command of: \n %s' %(prev_command)
			# print str_allowed_input

	# Set final GCS value. For safety, the GCS should start and end on this value. This value tells the UAV to follow the previous command. If no previous command exists, then this value tells the UAV to do nothing. 
	gcs.parameters['PIVOT_TURN_ANGLE'] = 100

	# Disarm and close GCS object before exiting script
	disarm_vehicle(gcs,'GCS')
	gcs.close()

	print('GCS program completed\n')