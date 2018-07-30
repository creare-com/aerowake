#!/usr/bin/env python2

import numpy as np
import os
import sys

'''
This function takes a list of North-East-Down (NED) coordinates from a specified file and rotates them in 2D space so that the North direction is aligned with the specified heading. The Down coordinates are unchanged by this function.

Usage:
	python rotate_mission.py <mission file dir> <mission file name> <new bearing>

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

def calculate_new_coords(brg,orig_coords):
	'''
	Rotates each coordinate in 2D space to be aligned with the specified bearing from North. Altitude (Down) is unchanged. 
	'''
	# Extract arrays
	N = orig_coords[0]
	E = orig_coords[1]
	D = orig_coords[2]

	# Check bearing given and convert to radians
	brg = float(brg)
	if brg < 0 or brg > 360.0:
		print 'Bearing must be between 0 and 360 degrees.'
		return orig_coords
	brg = brg*np.pi/180

	# Create rotation matrix
	R = [[np.cos(brg),-np.sin(brg)],[np.sin(brg),np.cos(brg)]]
	R = np.matrix(R)

	# Rotate NE parts and assign to new array
	new_coords = [[],[],list(D)]
	for i in range(0,len(N)):
		arr = np.array([[N[i]],[E[i]]])
		new_N,new_E = R*arr
		new_N = round(new_N,5)
		new_E = round(new_E,5)
		new_coords[0].append(new_N)
		new_coords[1].append(new_E)

	return new_coords

def write2py(coords,filename):
	'''
	Writes coords to specified file in directory that is current at time of calling this function. To match desired format of a mission file for Aerowake project, this function will write the new file as:

	# This mission file created from rotate_mission.py, a part of the Creare AeroWake project. 
	wp_N = [#,#,#,...]
	wp_E = [#,#,#,...]
	wp_D = [#,#,#,...]
	num_wp = #
	'''
	write_str = '# This mission file created using rotate_mission.py, a part of the Creare AeroWake project.\n\n'
	write_str = write_str + 'wp_N = %s\n' %(coords[0])
	write_str = write_str + 'wp_E = %s\n' %(coords[1])
	write_str = write_str + 'wp_D = %s\n' %(coords[2])
	write_str = write_str + 'num_wp = [len(wp_N)]'
	
	with open(filename, 'w') as output:
		output.write(write_str)

#-------------------------------------------------------------------------------
#
# Main Thread Start
#
#-------------------------------------------------------------------------------

if __name__ == '__main__':
	# Parse user input
	filedir = sys.argv[1]
	filename = sys.argv[2]
	brg = sys.argv[3]

	if not filedir[-1] == '/':
		filedir = filedir + '/'

	if not filename[-3:] == '.py':
		raise ValueError('File type must be .py')
	else:
		filename = filename[:-3]

	# Change directory to where coordinate file is stored
	with cd(filedir):
		'''
		While inside this 'with' block, we are in the user-specified directory.
		'''
		usr_file = __import__(filename)
		try:
			orig_coords = (usr_file.wp_N, usr_file.wp_E, usr_file.wp_D)
		except:
			raise NameError('Original file must define wp_N, wp_E, and wp_D')
		new_coords = calculate_new_coords(brg,orig_coords)
		new_filename = filename + '_rot.py'
		write2py(new_coords,new_filename)

	'''
	Once outside of the 'with' block, we are back in the original directory
	'''
	print 'Program completed. Find the new .py file at %s.' %(filedir + new_filename)

