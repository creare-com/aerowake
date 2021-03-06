import numpy as np
from rotate_mission import rotate, write2py

'''
Usage:
python mission.py

The mission is specified in Cartesian coordinates with respect to a North-East-Down (NED) coordinate frame centered at the GCS. The rotate_mission.py script can be used to rotate the mission to a specified bearing. 

The convention is as follows:
	- origin at GCS GPS module
  - North will be rotated to face aft
  - East will be rotated to port
  - Down will remain facing downwards

NOTE: If a variable is called 'alt', 'altitude', or some similar variation, then you should enter the desired height above ground. HOWEVER, the third axis cooresponds to down, which means a value of wp_D = -10 indicates that the UAV should fly 10 meters ABOVE the GCS altitude. The final wp_D list in mission_rot.py should be entirely negative.

NOTE: Takeoff and land are not waypoints.

Missions:
  1 Back and forth on a line at specified distances and altitudes
  2 Single-altitude arc at specified radius and altitude
  3 Double-altitude arc at specified radius and altitudes
'''

try:
  mission_str = "\nMissions:\n  1: Back and forth on E-W line (4 wp)\n  2: Single-altitude arc (8 wp)\n  3: Double-altitude arc (10 wp)\n"
  print mission_str

  load_mission = input("What mission do you want to fly? ")
  while load_mission not in [1, 2, 3]:
    print "You must enter a valid mission number."
    load_mission = input("What mission do you want to fly? ")

  offset = input("What is the offset? [m] ")
  while offset < 0:
    print "You must enter an offset >= 0."
    offset = input("What is the offset? [m] ")

  if load_mission == 1:
    radius = input("What distance away? [m] ")
    while radius < 5:
      print "Distance must be at least 5 meters."
      radius = input("What distance away? [m] ")
  else:
    radius = input("What radius? [m] ")
    while radius < 5:
      print "Radius must be at least 5 meters."
      radius = input("What radius? [m] ")

  alt_takeoff = input("What takeoff altitude? [m] ")
  while alt_takeoff <= 0:
    print "Takeoff altitude must be positive."
    alt_takeoff = input("What takeoff altitude? [m] ")

  alt_1 = input("What primary altitude? [m] ")
  while alt_1 <= 0:
    print "Primary altitude must be positive."
    alt_1 = input("What primary altitude? [m] ")

  if load_mission == 3:
    alt_2 = input("What secondary altitude? [m] ")
    while alt_2 <= 0:
      print "Secondary altitude must be positive."
      alt_2 = input("What secondary altitude? [m] ")

  if load_mission == 1:
    step_E = input("What horizontal spacing? [m] ")
    while step_E < 0:
      print "Horizontal spacing must be positive."
      step_E = input("What horizontal spacing? [m] ")

except KeyboardInterrupt as e:
  print '\n\nGot ^C. Cancelling mission creation.'
  load_mission = 0

except NameError as e:
  print "Enter only numbers."
  load_mission = 0



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
  alt    = alt_1 # [m]

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
  r   = radius
  alt = alt_1

  # Create mission
  step_E  = round(r*np.sin(np.pi/6)/2,2) # Horizontal distance between waypoints
  basic_E = [0, 1, 2, 1, 0, -1, -2, -1]
  num_wp  = 8
  wp_E    = [i*step_E for i in basic_E]
  wp_N    = get_N(wp_E,r)
  wp_D    = [-i*alt for i in np.ones(num_wp)]



'''
Mission 3: Mulit-alt arc of 10 waypoints at specified radius and alts. Altitude rises between 2 and 3, and falls between 7 and 8.

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
  r     = radius
  alt_1 = alt_1
  alt_2 = alt_2

  # Create mission
  step_E  = round(r*np.sin(np.pi/6)/2,2) # Horizontal distance between waypoints
  basic_E = [0, 1, 2, 2, 1, 0, -1, -2, -2, -1]
  basic_D = [0, 0, 0, 1, 1, 1,  1,  1,  0,  0]
  num_wp  = 10
  wp_E    = [i*step_E for i in basic_E]
  wp_N    = get_N(wp_E,r)
  wp_D    = [-1.0*alt_1 if i == 0 else -1.0*alt_2 for i in basic_D]



if not load_mission == 0:
  # Offset as desired
  wp_offset = offset # [m] Top view distance between GCS GPS and tether vertex

  # Write mission to base_mission.py
  filename = 'base_mission.py'
  coords = (wp_N,wp_E,wp_D)
  write2py(alt_takeoff, wp_offset, coords, filename)

  # Print the calculated mission
  print '\nFind the mission in %s' %(filename)
  print '\nYour mission:\n  Takeoff to %s meters\n  Offset North by %s meters\n  wp_N %s\n  wp_E %s\n  wp_D %s\n  num_wp %s' %(alt_takeoff,wp_offset,wp_N,wp_E,wp_D,num_wp)
