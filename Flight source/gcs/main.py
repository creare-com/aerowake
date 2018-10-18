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
str_help = ''' help
	Show page 1 of allowed inputs
 help2
	Show page 2 of allowed inputs
 help3
	Show page 3 of allowed inputs

'''

str_allowed_input_1 = '''

Allowed input 1/3:
 listen
	Tell UAV to start listening to commands
 arm
	Arm UAV throttle
 takeoff
	Takeoff
 <Waypoint Number> 
	Navigate to designated waypoint (0 - %d)
 reelin
	Reel tether in to landing length
'''  %(num_wp - 1) + str_help

str_allowed_input_2 = '''

Allowed input 2/3:
 rotate <Degrees>
	Rotates mission by specified angle (-%s through %s, CW positive)
 alt[./,]
	Increases/Decreases mission altitude by 1 meter
 reel[./,]
	Increases/Decreases tether length by 0.5 meters
 wpspeed[./,]
	Increases/Decreases WPNAV_SPEED by 0.5 m/s
 wpaccel[./,]
	Increases/Decreases WPNAV_ACCEL by 0.3 m/s/s
'''  %(max_deg, max_deg) + str_help

str_allowed_input_3 = '''

Allowed input 3/3:
 disarm
	Disarm UAV throttle
 clear
	Clear current waypoint
 rgetdata
	Report tether info
 reelreset
	Reel in to 0 tether length
 cfae
	Clear a faulted motor controller and renable the reel
 land
	Land
 quit
	End GCS\'s main.py
''' + str_help

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
	print str_allowed_input_1

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
				logger.info(str_allowed_input_1)
				continue

			elif user_in == 'help2':
				invalid_input = False
				logger.info(str_allowed_input_2)
				continue

			elif user_in == 'help3':
				invalid_input = False
				logger.info(str_allowed_input_3)
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
				logger.info('Commanding UAV to listen to commands')
				prev_command = 'Command UAV to listen to commands'

			elif user_in == 'rgetdata':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to report data.')
					reel_data = get_reel_data()
					logger.info(' Reel at: %s',reel_data)
				else:
					logger.info('rgetdata called when not using reel.')
			
			elif user_in == 'reelin':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to reel in to landing.')
					commands_to_reel.put({"cmd":"goto", "L":0.4})
				else:
					logger.info('reelin called when not using reel.')

			elif user_in == 'reelreset':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to zero tether length.')
					commands_to_reel.put({"cmd":"goto", "L":0})
				else:
					logger.info('reelreset called when not using reel.')

			elif user_in[0:4] == 'reel':
				invalid_input = False
				curr_length = get_reel_data()['L']
				try:
					reel_change = 0
					flag_change_length = True
					for char in user_in[4:]:
						if char == '.':
							reel_change = reel_change + 0.5
						elif char == ',':
							reel_change = reel_change - 0.5
						else:
							flag_change_length = False
					if flag_change_length:
						if reel_change > 0:
							logger.info('Commanding reel to increase tether length by %s meters' %(reel_change))
							new_length = curr_length + reel_change
						elif reel_change < 0:
							logger.info('Commanding reel to decrease tether length by %s meters' %(reel_change))
							new_length = curr_length + reel_change
						else:
							new_length = curr_length
						commands_to_reel.put({"cmd":"goto", "L":new_length})
					else:
						logger.info('Invalid input. To increase tether length, enter \'reel.\'. To decrease tether length, enter \'reel,\'.')
				except TypeError as e:
					logger.info('Failed to get data from reel. Try again.')

			elif user_in == 'clearfault':
				invalid_input = False
				if using_reel:
					logger.info('Commanding reel to clear fault.')
					commands_to_reel.put({"cmd":"clearfault"})
				else:
					logger.info('clearfault called when not using reel.')

			elif user_in == 'cfae':
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

			elif user_in == 'arm':
				invalid_input = False
				# UAV knows that 359 means arm
				gcs.parameters[cmd_param] = 359
				logger.info('Commanding UAV to Arm')
				prev_command = 'Command UAV to Arm'

			elif user_in == 'disarm':
				invalid_input = False
				# UAV knows that 358 means disarm
				gcs.parameters[cmd_param] = 358
				logger.info('Commanding UAV to Disarm')
				prev_command = 'Command UAV to Disarm'

			elif user_in == 'takeoff':
				invalid_input = False
				# UAV knows that 357 means takeoff
				gcs.parameters[cmd_param] = 357
				logger.info('Commanding UAV to Takeoff to %s meters' %(alt_takeoff))
				if using_reel:
					logger.info('Commanding reel to %0.1f meters', alt_takeoff*safety_factor)
					commands_to_reel.put({"cmd":"goto", "L":alt_takeoff*safety_factor})
				prev_command = 'Command UAV to Takeoff'

			elif user_in == 'land':
				invalid_input = False
				# UAV knows that 356 means land
				gcs.parameters[cmd_param] = 356
				logger.info('Commanding UAV to Land')
				prev_command = 'Command UAV to Land'

			elif user_in == 'clear':
				invalid_input = False
				# UAV knows that 102 means clear
				gcs.parameters[cmd_param] = 102
				logger.info('Commanding UAV to clear the current waypoint')
				prev_command = 'Command UAV to clear the current waypoint'

			elif user_in[0:6] == 'rotate':
				invalid_input = False
				attempt_rotate = True
				# Instruct UAV to rotate current mission by specified bearing
				try:
					bearing = int(user_in[7:])
				except (ValueError, TypeError) as e:
					logger.info('Invalid rotation input. Please try again.')
					attempt_rotate = False
				if attempt_rotate:
					if bearing < -max_deg or bearing > max_deg:
						logger.info('Invalid rotation value. Rotation is relative and must be between -%s and %s degrees.' %(max_deg,max_deg))
					else:
						logger.info('Commanding UAV to rotate mission by %s degrees (CW pos)' %(bearing))
						gcs.parameters[bearing_param] = bearing
						gcs.parameters[cmd_param] = 355
						prev_command = 'Command UAV to rotate bearing by %s degrees' %(bearing)

			elif user_in[0:3] == 'alt':
				invalid_input = False
				increase_alt = False
				decrease_alt = False
				if user_in[3] == '.':
					increase_alt = True
				elif user_in[3] == ',':
					decrease_alt = True
				else:
					logger.info('Invalid input. To increase altitude, enter \'alt.\'. To decrease, enter \'alt,\'.')
				if increase_alt:
					logger.info('Commanding UAV to increase mission altitude by 1 meter')
					gcs.parameters[cmd_param] = 200
					# NOTE: subtracting since coordinate system is NED (i.e. negative values for altitude)
					gcs_mission[2] = [gcs_mission[2][i]-1 for i in range(0,len(gcs_mission[2]))]
					logger.info('New target altitude: %s' %(gcs_mission[2]))
				if decrease_alt:
					logger.info('Commanding UAV to decrease mission altitude by 1 meter')
					gcs.parameters[cmd_param] = 201
					# NOTE: adding since coordinate system is NED (i.e. negative values for altitude)
					gcs_mission[2] = [gcs_mission[2][i]+1 for i in range(0,len(gcs_mission[2]))]
					logger.info('New target altitude: %s' %(gcs_mission[2]))

			elif user_in[0:7] == 'wpspeed':
				invalid_input = False
				increase_nav_speed = False
				decrease_nav_speed = False
				if user_in[7] == '.':
					increase_nav_speed = True
				elif user_in[7] == ',':
					decrease_nav_speed = True
				else:
					logger.info('Invalid input. To increase waypoint navigation speed, enter \'wpspeed.\'. To decrease, enter \'wpspeed,\'.')
				if increase_nav_speed:
					logger.info('Commanding UAV to increase WPNAV_SPEED by 0.5 m/s')
					gcs.parameters[cmd_param] = 202
				if decrease_nav_speed:
					logger.info('Commanding UAV to decrease WPNAV_SPEED by 0.5 m/s')
					gcs.parameters[cmd_param] = 203

			elif user_in[0:7] == 'wpaccel':
				invalid_input = False
				increase_nav_accel = False
				decrease_nav_accel = False
				if user_in[7] == '.':
					increase_nav_accel = True
				elif user_in[7] == ',':
					decrease_nav_accel = True
				else:
					logger.info('Invalid input. To increase waypoint navigation acceleration, enter \'wpaccel.\'. To decrease, enter \'wpspeed,\'.')
				if increase_nav_accel:
					logger.info('Commanding UAV to increase WPNAV_ACCEL by 0.3 m/s/s')
					gcs.parameters[cmd_param] = 204
				if decrease_nav_accel:
					logger.info('Commanding UAV to decrease WPNAV_ACCEL by 0.3 m/s/s')
					gcs.parameters[cmd_param] = 205

			else:
				try:
					user_in = int(user_in)
					# If user enters an integer, set the chosen GCS parameter to that integer value. The UAV will read this parameter and navigate to that waypoint. E.g. if user enters 2, then UAV will navigate to the waypoint at index 2.
					if user_in >= 0 and user_in < num_wp:
						invalid_input = False
						logger.info('Commanding UAV to waypoint %s' %(user_in))
						gcs.parameters[cmd_param] = user_in
						# Command reel to desired length based on current waypoint
						d_aft = gcs_mission[0][user_in]
						d_port = gcs_mission[1][user_in]
						d_down = gcs_mission[2][user_in]
						logger.debug('(d_aft,d_port, d_down) = (%.01f,%.01f,%.01f)'%(d_aft,d_port, d_down))
						if using_reel:
							dist = math.sqrt((d_aft**2) + (d_port**2) + (d_down**2))
							logger.info('Commanding reel to %s meters', dist*safety_factor)
							commands_to_reel.put({"cmd":"goto", "L":dist*safety_factor})
						prev_command = 'Command UAV to waypoint %s' %(user_in)
				except ValueError:
					# Will be invalid input and will print such as defined below
					pass

			if invalid_input:
				logger.info('Invalid input. Type \'help\' for allowed commands.')
				logger.info(' UAV is following previous command of:  %s' %(prev_command))

			time.sleep(0.5)
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
