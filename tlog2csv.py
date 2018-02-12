#!/usr/bin/env python2

from dronekit import LocationGlobalRelative
from pymavlink import mavutil
import csv
import sys
import os

'''
This script takes a MAVLink telemetry log file (*.tlog) and outputs a CSV file of the same name to the same directory. 
'''

#-------------------------------------------------------------------------------
#
# Classes and Helper Functions
#
#-------------------------------------------------------------------------------

class cd(object):
	'''
	Context manager for changing the current working directory

	Written by Brian M. Hunt. Code can be found at:
	https://stackoverflow.com/questions/431684/how-do-i-cd-in-python/13197763#13197763
	'''
	def __init__(self, newPath):
		self.newPath = os.path.expanduser(newPath)

	def __enter__(self):
		self.savedPath = os.getcwd()
		os.chdir(self.newPath)

	def __exit__(self, etype, value, traceback):
		os.chdir(self.savedPath)

class data:
	'''
	Serves as a more specific data carrier for the data required by the Aerowake project since dictionaries are a bit cumbersome for this project.
	'''
	def __init__(self, data_dict):
		self.__data = {key:tuple(v for v in value) for key, value in data_dict.items()}
		self.__type = self.__data['mavpackettype'][0]
		del self.__data['mavpackettype']
		self.__fields = self.__data.keys()

	def get_type(self):
		return self.__type

	def get_fields(self):
		return self.__fields

	def get_values(self,param):
		if param in self.__fields:
			return self.__data[param]
		else:
			print 'Invalid parameter requested: %s' %(param)
			return None

def get_data(filename,param):
	'''
	Takes telemetry log (*.tlog) file, extracts relevant information, and stores it in a CSV file of the same name.

	Modified version of method from 'flight_replay.py' example from:
	http://python.dronekit.io/examples/flight_replay.html

	Allowed parameters found by loading *.tlog in APMPlanner:

	Known allowed functions on messages can be found at: 
	http://pydoc.net/pymavlink/2.2.7/pymavlink.dialects.v20.ardupilotmega/

	Useful functions on messages are:
	- m = mlog.recv_match(type=[param]) # Obtain a message from log file
	- m.get_fieldnames()
	- m.get_type()
	- m.to_dict()
	'''

	allowed_params = ['AHRS','AHRS2','ATTITUDE','BATTERY_STATUS','GLOBAL_POSITION_INT','GPS_GLOBAL_ORIGIN','GPS_RAW_INT','HEARTBEAT','HWSTATUS','LOCAL_POSITION_NED','NAV_CONTROLLER_OUTPUT','POWER_STATUS','RAW_IMU','RC_CHANNELS','RC_CHANNELS_RAW','SCALED_IMU2','SCALED_PRESSURE','SENSOR_OFFSETS','SERVO_OUTPUT_RAW','SIMSTATE','SYS_STATUS','TERRAIN_REPORT','VFR_HUD']

	if param not in allowed_params:
		raise ValueError('Invalid parameter.')

	messages = []
	mlog = mavutil.mavlink_connection(filename)
	while True:
	# for i in range(0,10):
		try:
			m = mlog.recv_match(type=[param])
			if m is None:
				break
		except Exception:
			break
		messages.append(m)
	return messages

def extract_data(ds):
	'''
	Takes a list of dictionary objects and returns a single dictionary object. Each dictionary in the original list must have the exact same key values. The returned dictionary also has these key values.

	The ordering of the list is preserved.

	E.g.:
		ds = [{'vx': 0, 'lon': 2},{'vx': 1, 'lon': 3}]
		d  = [{'vx': [0,1], 'lon': [2,3]}]
	'''
	d = {}
	for k in ds[0].keys():
		d[k] = [x[k] for x in ds]
	return d

def write2csv(param_dict,filename):
	'''
	Takes in a dictionary containing parameter types as keys and data class objects as values. Each field in each data class is written to the current working directory as a .csv file with the name specified by filename.

	Steps:
		to_write = []
		max_length = 0
		for each param in param_dict
			switch-case to extract relevant information from that parameter
			to_write.append(['param.field',field_data]) for each field in param
		pad elements of to_write with empties so all elements are same length
		for i in range(0,len(lists in to_write)):
			write_row(to_write[i])
	'''

	to_write = []
	max_length = 0
	for key,data_obj in sorted(param_dict.items()):
		for field in sorted(data_obj.get_fields()):
			name = key + '.' + field
			data = data_obj.get_values(field)
			write_list = [name] + [elm for elm in data]
			to_write.append(write_list)
			if len(to_write[-1]) > max_length:
				max_length = len(to_write[-1])

	# Pad each list of data with extra spaces so all are same length
	for sublist in to_write:
		while len(sublist) < max_length:
			sublist.append('')

	# Remove extension, if any, from given filename and rename with .csv extension
	i = filename.find('.')
	if i > 0:
		filename = filename[:i]
	filename = filename + '.csv'

	# Write to CSV file
	with open(filename, 'w') as csvfile:
		spamwriter = csv.writer(csvfile,delimiter=',')

		for i in range(0,max_length):
			row_to_write = [item[i] for item in to_write]
			spamwriter.writerow(row_to_write)

#-------------------------------------------------------------------------------
#
# Main Thread Start
#
#-------------------------------------------------------------------------------

if __name__ == '__main__':
	# Parse user input
	filedir = sys.argv[1]
	filename = sys.argv[2]

	# List parameter types to extract
	desired_params = [
		'ATTITUDE',
		'GLOBAL_POSITION_INT',
		# 'HEARTBEAT',
		'LOCAL_POSITION_NED',
		'RAW_IMU',
		# 'SIMSTATE',
		# 'SYS_STATUS'
		]

	# Change directory to where the *.tlog is stored
	with cd(filedir):
		'''
		While inside this 'with' block, we are in the user-specified directory.
		'''
		all_params = {}
		for param in desired_params:
			m_list = get_data(filename,param)
			m_list_of_dicts = [m.to_dict() for m in m_list]
			param_data = data(extract_data(m_list_of_dicts))
			all_params[param] = param_data
		write2csv(all_params,filename)

	'''
	Once outside of the 'with' block, we are back in the original directory
	'''

	# Remove extension, if any, from given filename and rename with .csv extension
	i = filename.find('.')
	if i > 0:
		filename = filename[:i]
	filename = filename + '.csv'
	new_file_path = filedir + filename

	print 'Program completed. Find the new CSV file at %s.' %(new_file_path)




