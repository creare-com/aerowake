#!/usr/bin/env python2

# Required for importing mission and helper functions
import sys
sys.path.append('../')

# Required for messaging the drone
from dronekit import connect
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

# Set connection path to GCS
if len(sys.argv) > 1 and sys.argv[1] == 'sim':
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

# Define a string that will print to show allowed user input
str_allowed_input = '\n\nAllowed input:\n listen\n  Tell UAV to start listening to commands\n arm\n  Arm UAV throttle\n disarm\n  Disarm UAV throttle\n takeoff\n  Takeoff\n <Waypoint Number> \n  Navigate to designated waypoint (0 - %d)\n clear\n  Clear current waypoint\n rotate <Degrees>\n  Rotates mission by specified angle (-%s through %s, CW positive)\n rgetdata\n  Report tether info\n land\n  Land\n help\n  Show this list of allowed inputs\n end\n  Ask UAV to end its main.py\n quit\n  End GCS\'s main.py\n\n' %(num_wp - 1, max_deg, max_deg)

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
	logger = logging.getLogger('gcs_logger')
	logger.setLevel(logging.DEBUG)
	# Create file handler that sends all logger messages (DEBUG and above) to file
	logfile = '%s/logs/gcs-logs/gcs-%s.log' %(os.path.expanduser('~'),time.strftime('%Y-%m-%d-%Hh-%Mm-%Ss', time.localtime()))
	fh = logging.FileHandler(logfile)
	print 'Logging to %s' %(logfile)
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
	logger.info('Waiting for GCS\n')
	while True:
		try:
			gcs = connect(gcs_connect_path, baud = gcs_baud, heartbeat_timeout = 60, rate = 20, wait_ready = True)
			break
		except Exception as e:
			logger.critical('GCS failed to connect with message: %s' %(e.message))
			raise e

	logger.info('GCS pixhawk connected to GCS\n')

	#------------------------------------
	# Listeners for Logging
	#------------------------------------

	@gcs.on_message('SYSTEM_TIME')
	def time_callback(self,attr_name, msg):
		logger.debug('gcsGPSTIME,%s' %msg)

	@gcs.on_message('LOCAL_POSITION_NED')
	def local_position_NED_callback(self,attr_name, msg):
		logger.debug('localPosNED,%s' %msg)

	# ack_param is used as an acknowledge parameter by the UAV.
	@gcs.parameters.on_attribute(ack_param)
	def UAV_parameter_callback(self, attr_name, UAV_param):
		if UAV_param == 103:
			# attempts to end uav main.py
			logger.info('UAV received command to end its main.py.\n')
		elif UAV_param == 102:
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
		elif UAV_param >= 0 and UAV_param < num_wp:
			# commanding waypoint
			logger.info('UAV received command to go to waypoint %d.\n', UAV_param)
		else:
			# something went wrong
			logger.warning('WARNING: UAV acknowledged an unknown command.\n')

	logger.info('------------------SYSTEM IS READY!!------------------')
	logger.info('-----------------------------------------------------\n')



	#-----------------------------------------------------------------------------
	#
	# Control Code
	#
	#-----------------------------------------------------------------------------

	if using_reel:
		# Start reel controller
		commands_to_reel = Queue()
		data_from_reel = Queue()
		try:
			reel = reel_run(commands_to_reel, data_from_reel)
		except Exception:
			logging.critical('Problem connecting to reel. Aborting.')
			setup_abort("Reel System Failure")
		reel.start()

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
		103			UAV will end its main.py if it is disarmed
		359     UAV will arm
		358     UAV will disarm
		357     UAV will takeoff to a pre-programmed height
		356     UAV will land according to its landing protocol
		355		  UAV will rotate to bearing given by parameter RNGFND_TURN_ANGL
		0+      UAV will navigate to the waypoint at the index specified
						The acceptable waypoint indices are 0 through num_wp - 1

	If an invalid input is entered, such as a typo or a waypoint number that does not exist, the UAV will follow the previous command. The GCS will echo this behavior to the terminal, notifying the user of the UAV's behavior in the event of an invalid input.

	When the quit command is given, the UAV will continue to follow its most recent command.
	'''

	user_in = None
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
			if user_in == 'help':
				invalid_input = False
				logger.info(str_allowed_input)
				continue

			elif user_in == 'quit':
				invalid_input = False
				# Will terminate on next loop. UAV will continue with previous command
				logger.info('Terminating GCS loop.')
				logger.info(' UAV is following previous command of: \n %s' %(prev_command))
				continue

			elif user_in == 'listen':
				invalid_input = False
				# UAV knows that 101 means start listening to commands
				gcs.parameters[cmd_param] = 101
				logger.info('Commanding UAV to listen to commands\n')
				prev_command = 'Command UAV to listen to commands\n'

			elif user_in == 'rgetdata':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to report data.')
					reel_data = get_reel_data()
					logger.info(' Reel at: %s\n',reel_data)
				else:
					logger.info('rgetdata called when not using reel.\n')

			elif user_in == 'arm':
				invalid_input = False
				# UAV knows that 359 means arm
				gcs.parameters[cmd_param] = 359
				logger.info('Commanding UAV to Arm\n')
				prev_command = 'Command UAV to Arm\n'

			elif user_in == 'disarm':
				invalid_input = False
				# UAV knows that 358 means disarm
				gcs.parameters[cmd_param] = 358
				logger.info('Commanding UAV to Disarm\n')
				prev_command = 'Command UAV to Disarm\n'

			elif user_in == 'takeoff':
				invalid_input = False
				# UAV knows that 357 means takeoff
				gcs.parameters[cmd_param] = 357
				logger.info('Commanding UAV to Takeoff to %s meters\n' %(alt_takeoff))
				if using_reel:
					logger.info('Commanding reel to %s meters\n', alt_takeoff*safety_factor)
					commands_to_reel.put({"cmd":"goto", "L":alt_takeoff*safety_factor})
				prev_command = 'Command UAV to Takeoff\n'

			elif user_in == 'land':
				invalid_input = False
				# UAV knows that 356 means land
				gcs.parameters[cmd_param] = 356
				logger.info('Commanding UAV to Land\n')
				prev_command = 'Command UAV to Land\n'

			elif user_in == 'end':
				invalid_input = False
				# UAV knows that 103 means attempt to stop UAV main.py
				gcs.parameters[cmd_param] = 103
				logger.info('Commanding UAV to stop its main.py\n')
				prev_command = 'Command UAV to stop its main.py\n'

			elif user_in == 'clear':
				invalid_input = False
				# UAV knows that 102 means clear
				gcs.parameters[cmd_param] = 102
				logger.info('Commanding UAV to clear the current waypoint\n')
				prev_command = 'Command UAV to clear the current waypoint\n'

			elif user_in[0:6] == 'rotate':
				invalid_input = False
				attempt_rotate = True
				# Instruct UAV to rotate current mission by specified bearing
				try:
					bearing = int(user_in[7:])
				except (ValueError, TypeError) as e:
					logger.info('Invalid rotation input. Please try again.\n')
					attempt_rotate = False
				if attempt_rotate:
					if bearing < -max_deg or bearing > max_deg:
						logger.info('Invalid rotation value. Rotation is relative and must be between -%s and %s degrees.\n' %(max_deg,max_deg))
					else:
						logger.info('Commanding UAV to rotate bearing to %s\n' %(bearing))
						gcs.parameters[bearing_param] = bearing
						gcs.parameters[cmd_param] = 355
						prev_command = 'Command UAV to rotate bearing by %s degrees\n' %(bearing)

			else:
				try:
					user_in = int(user_in)
					# If user enters an integer, set the chosen GCS parameter to that integer value. The UAV will read this parameter and navigate to that waypoint. E.g. if user enters 2, then UAV will navigate to the waypoint at index 2.
					if user_in >= 0 and user_in < num_wp:
						invalid_input = False
						logger.info('Commanding UAV to waypoint %s\n' %(user_in))
						gcs.parameters[cmd_param] = user_in
						# Command reel to desired length based on current waypoint
						d_aft = base_mission.wp_N[user_in]
						d_port = base_mission.wp_E[user_in]
						d_down = base_mission.wp_D[user_in]
						logger.debug('(d_aft,d_port, d_down) = (%.01f,%.01f,%.01f)'%(d_aft,d_port, d_down))
						if using_reel:
							dist = math.sqrt((d_aft**2) + (d_port**2) + (d_down**2))
							logger.info('Commanding reel to %s meters\n', dist*safety_factor)
							commands_to_reel.put({"cmd":"goto", "L":dist*safety_factor})
						prev_command = 'Command UAV to waypoint %s\n' %(user_in)
				except ValueError:
					# Will be invalid input and will print such as defined below
					pass

			if invalid_input:
				logger.info('Invalid input. Type \'help\' for allowed commands.')
				logger.info(' UAV is following previous command of: \n %s' %(prev_command))

	except KeyboardInterrupt:
		logger.info('\nGot CTRL+C. Cleaning up and exiting.\n')

	#------------------------------------
	# Outside of while loop
	#------------------------------------

	# Set final GCS value. For safety, the GCS should start and end on this value. This value tells the UAV to follow the previous command. If no previous command exists, then this value tells the UAV to do nothing.
	gcs.parameters[cmd_param] = 100

	# Close GCS object before exiting script
	gcs.close()

	if using_reel:
		# Shutdown reel controller
		commands_to_reel.put({'cmd':'exit'})
		reel.join()

	logger.info('GCS program completed\n')
