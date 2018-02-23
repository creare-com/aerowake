import numpy as np

'''
The mission is specified in Cartesian coordinates with respect to a North-East-Down (NED) coordinate frame centered at the GCS. The rotate_mission.py script can be used to rotate the mission to a specified bearing. 

The convention is as follows:
	- origin at GCS GPS module
	- x coordinate goes East
	- y coordinate goes North
	- z coordinate goes Down 

NOTE: z cooresponds to down, which means a value of z = -10 indicates that the UAV should fly 10 meters above the GCS altitude.
'''

# # Mission 1: Back and forth on east-to-west line at 10 m alt
# wp_N = [ 20,  20,  20,  20,  20]
# wp_E = [  0,  10,   0, -10,   0]
# wp_D = [-10, -10, -10, -10, -10]
# num_wp = [len(wp_N)]

# # Mission 2: Back and forth on east-to-west line at 1 m alt
# wp_N = [20, 20, 20,  20, 20]
# wp_E = [ 0, 10,  0, -10,  0]
# wp_D = [-1, -1, -1,  -1, -1]
# num_wp = [len(wp_N)]

# Mission 3: Simulate a 20 m arc traveling east-to-west at 10 m alt
def get_y3(x,r):
	'''
	Returns the y coordinate corresponding with the given x coordinate on an arc of radius r centered at the origin.
	'''
	return  round(np.sqrt(r**2 - x**2),2)
basic_E = [2.5,5,7.5,10,12.5]
basic_E_neg = [-x for x in basic_E]
center_E = [0.0]
end_E = [15]
end_E_neg = [-15]
wp_E = list(center_E) + list(basic_E) + list(end_E) + list(reversed(basic_E)) + list(center_E) + list(basic_E_neg) + list(end_E_neg) + list(reversed(basic_E_neg)) + list(center_E)
wp_N = [get_y3(x,20) for x in wp_E]
wp_D = [-10]*len(wp_N)
num_wp = [len(wp_N)]
