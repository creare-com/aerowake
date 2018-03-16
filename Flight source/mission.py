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

# # Mission 4: Abbreviated arc at Brigg's field
# def get_y4(x,r):
# 	return round(np.sqrt(r**2 - x**2),2)
# wp_E = [0, 4.5,8.5,4.5,0,-4.5,-8.5,-4.5,0]
# wp_N = [get_y4(x,12) for x in wp_E]
# wp_D = [-2]*len(wp_N)
# num_wp = [len(wp_N)]

# Mission 5: Abbreviated arc at stadium
def get_y5(x,r):
	return round(np.sqrt(r**2 - x**2),2)
wp_E = [0, 5.67,10.6,5.67,0,-5.67,-10.6,-5.67,0]
wp_N = [get_y5(x,15) for x in wp_E]
wp_D = [-1]*len(wp_N)
num_wp = [len(wp_N)]

# Mission 6: Arc on turf
def get_y6(x,r):
	return round(np.sqrt(r**2 - x**2),2)
wp_E = [0, -6.5, -11.1, -15.7, -11.1, -6.5, 0]
wp_N = [get_y6(x,20) for x in wp_E]
wp_D = [-1]*len(wp_N)
num_wp = [len(wp_N)]
