

import pose_control



p = pose_control.pose_controller_class()


NE = [42.36,-71.08]
NW = [42.36,-71.083]
SE = [42.357,-71.08]
SW = [42.357,-71.083]

# NE to NW: 247 m
# NE to SW: 416 m
# NE to SE: 332 m
# NW to SW: 334 m
# SW to SE: 247 m

p.uav_coord = SE
p.uav_alt = 100
p.uav_heading = 0

p.gcs_coord = NE
p.gcs_alt = 0
p.gcs_heading = 0

p.goal_pose = [0,0,0]

p.gcs_tether_l = 0 

print "distance: %f" %p.get_distance()

print "diag distance: %f" %p.get_diagonal_distance()

print "bearing: %f " %p.get_bearing()

print (p.get_relative_angles())

print p.run_pose_controller()