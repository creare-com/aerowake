#!/usr/bin/env python2

import logging
import mission
import time
import sys
sys.path.append('../')
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
num_wp = len(mission.THETA)

# Define a string that will print to show allowed user input
str_allowed_input = '\n\nAllowed input:\n Waypoint Number < %d (0-indexed)\n  Navigate to designated waypoint\n takeoff\n  Command UAV to takeoff\n land\n  Command UAV to land\n quit\n  Terminate program\n\n' %(num_wp)

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

	logging.info("------------------SYSTEM IS READY!!------------------")
	logging.info("-----------------------------------------------------")

	#-----------------------------------------------------------------------------
	#
	# Control Code
	#
	#-----------------------------------------------------------------------------

	arm_vehicle(gcs,'GCS')

	print str_allowed_input

	# Set default parameter value. UAV knows that 100 means 'do nothing'.
	gcs.parameters['PIVOT_TURN_ANGLE'] = 100
	user_in = None
	while not user_in == 'quit':

		print 'param: ', gcs.parameters['PIVOT_TURN_ANGLE']

		# Wait for user input
		print '\nEnter command:'
		user_in = raw_input()

		# Adjust user input for conditional statements
		user_in = user_in.lower()

		# Perform some action based upon the user input
		invalid_input = True
		if user_in.isalnum():
			if user_in == 'takeoff':
				# UAV knows that 359 means takeoff
				gcs.parameters['PIVOT_TURN_ANGLE'] = 359
				invalid_input = False
				print 'Commanding UAV to Takeoff'
			elif user_in == 'land':
				# UAV knows that 358 means land
				gcs.parameters['PIVOT_TURN_ANGLE'] = 358
				invalid_input = False
				print 'Commanding UAV to Land'
			elif user_in == 'quit':
				# Will terminate on next loop. UAV will continue with previous command
				print 'Terminating GCS loop. UAV will follow previous command.'
				invalid_input = False
			else:
				try:
					user_in = int(user_in)
					# If user enters an integer, set the chosen GCS parameter to that integer value. The UAV will read this parameter and navigate to that waypoint. E.g. if user enters 2, then UAV will navigate to the waypoint at index 2. 
					if user_in >= 0 and user_in < num_wp:
						print "Commanding UAV to waypoint %s" %(user_in)
						gcs.parameters['PIVOT_TURN_ANGLE'] = user_in
						invalid_input = False
				except ValueError:
					# Will be invalid input and will print str_allowed_input
					pass

		if invalid_input:
			print 'Invalid input.'
			# print str_allowed_input

	# Disarm and close GCS object before exiting script
	disarm_vehicle(gcs,'GCS')
	gcs.close()

	print("Completed")