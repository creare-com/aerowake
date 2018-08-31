import numpy as np

'''
The mission is specified in Cartesian coordinates with respect to a North-East-Down (NED) coordinate frame centered at the GCS. The rotate_mission.py script can be used to rotate the mission to a specified bearing. 

The convention is as follows:
	- origin at GCS GPS module

NOTE: If a variable is called 'alt', 'altitude', or some similar variation, then you should enter the desired height above the ground station. HOWEVER, the third axis cooresponds to down, which means a value of wp_D = -10 indicates that the UAV should fly 10 meters ABOVE the GCS altitude. The final wp_D list should be entirely negative.

NOTE: Takeoff and land are not waypoints.

Missions:
  1 Back and forth on a line at specified distances and altitudes
  2 Single-altitude arc at specified radius and altitude
  3 Double-altitude arc at specified radius and altitudes
'''

load_mission = 3 # Index of mission to load
alt_low      = 5
radius       = 20
step_E       = 5 # Only needed if loading mission 1
alt_high     = 8 # Only needed if loading mission 3



def get_N(wp_E,r):
	'''
	Returns the wp_N coordinate list corresponding with the given wp_E coordinate list on a spherical arc of radius r centered at the origin.
	'''
	return [round(np.sqrt(r**2 - E**2),2) for E in wp_E]



'''
Mission 1: Simple line of 4 waypoints on E axis at specified dist and alt

      step_E
      <-->

3---0/2---1  
     |       ^
     |       | dist_N
     |       |
     |       V
     G 
'''

if load_mission == 1:
  # Set characteristics
  dist_N = radius # [m]
  step_E = step_E # [m]
  alt    = alt_low # [m]

  # Create mission
  dist_N  = [dist_N] # Change to list for list multiplication
  basic_E = [0, 1, 0, -1]
  num_wp  = 4
  wp_N    = dist_N*num_wp
  wp_E    = [i*step_E for i in basic_E]
  wp_D    = [-i*alt for i in np.ones(num_wp)]



'''
Mission 2: Simple arc of 8 waypoints at specified radius and alt.

        --0/4--            
  --5/7-   |   -1/3--     ^
6-         |         -2   |
           |              | radius, r
           |              |
           |              |
           |              V
           G
'''

if load_mission == 2:
  # Set characteristics
  r   = 20
  alt = 5

  # Create mission
  step_E  = round(r*np.cos(np.pi/4)/2,2) # Horizontal distance between waypoints
  basic_E = [0, 1, 2, 1, 0, -1, -2, -1]
  num_wp  = 8
  wp_E    = [i*step_E for i in basic_E]
  wp_N    = get_N(wp_E,r)
  wp_D    = [-i*alt for i in np.ones(num_wp)]



'''
Mission 2: Mulit-alt arc of 10 waypoints at specified radius and alts. Altitude rises between 2 and 3, and falls between 7 and 8.

          --0/5--            
    --6/9-   |   -1/4--       ^
7/8-         |         -2/3   |
             |                | radius, r
             |                |
             |                |
             |                V
             G
'''

if load_mission == 3:
  # Set characteristics
  r        = radius
  alt_low  = alt_low
  alt_high = alt_high

  # Create mission
  step_E  = round(r*np.cos(np.pi/4)/2,2) # Horizontal distance between waypoints
  basic_E = [0, 1, 2, 2, 1, 0, -1, -2, -2, -1]
  basic_D = [0, 0, 0, 1, 1, 1,  1,  1,  0,  0]
  num_wp  = 10
  wp_E    = [i*step_E for i in basic_E]
  wp_N    = get_N(wp_E,r)
  wp_D    = [1.0*alt_low if i == 0 else 1.0*alt_high for i in basic_D]
