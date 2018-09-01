#!/usr/bin/env python2

import numpy as np
import os
import sys

'''
This function takes a list of North-East-Down (NED) coordinates from a specified file and rotates them in 2D space so that the North direction is aligned with the specified heading. The Down coordinates are unchanged by this function.

Usage:
	python rotate_mission.py des_bearing

'''

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
		raise ValueError('Bearing must be between 0 and 360 degrees.')
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

def write2py(alt_takeoff,coords,filename):
	'''
	Writes coords to specified filename. To match desired format of a mission file for Aerowake project, this function will write the new file as:

	# This mission file created from rotate_mission.py, a part of the Creare AeroWake project. 
	wp_N = [#,#,#,...]
	wp_E = [#,#,#,...]
	wp_D = [#,#,#,...]
	num_wp = #
	'''

	write_str = '# This mission file created using rotate_mission.py, a part of the Creare AeroWake project.\n\n'
	write_str = write_str + 'alt_takeoff = %s\n' %(alt_takeoff)
	write_str = write_str + 'wp_N = %s\n' %(coords[0])
	write_str = write_str + 'wp_E = %s\n' %(coords[1])
	write_str = write_str + 'wp_D = %s\n' %(coords[2])
	write_str = write_str + 'num_wp = [len(wp_N)]'
	
	with open(filename, 'w') as output:
		output.write(write_str)

def rotate(orig_coords, bearing):
	return calculate_new_coords(bearing,orig_coords)
