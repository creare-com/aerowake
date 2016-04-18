

import pose_control
import numpy as np

# phi_in= -1
# th_in = 0

# phi = 0  * np.pi/180
# th = 45   * np.pi/180

# fix = -phi_in*np.sin(phi) + th_in*np.cos(th)*np.cos(phi)
# fiy = phi_in*np.cos(phi) + th_in*np.cos(th)*np.sin(phi)
# fiz = -th_in*np.sin(th) 
# print "\n \n"
# print "%.2f  %.2f  %.2f"  %(fix,fiy,fiz)
# print "\n \n"
# exit()

p = pose_control.pose_controller_class()


# NE = [42.36,-71.08]
# NW = [42.36,-71.083]
# SE = [42.357,-71.08]
# SW = [42.357,-71.083]
# # NE to NW: 247 m
# # NE to SW: 416 m
# # NE to SE: 332 m
# # NW to SW: 334 m
# # SW to SE: 247 m

NE = [42.36,-71.08]
NW = [42.36,-71.081]
SE = [42.359,-71.08]
SW = [42.359,-71.081]

p.uav_coord = SE
p.uav_alt = 10
p.uav_heading = 0

p.gcs_coord = NE
p.gcs_alt = 0
p.gcs_heading = 0

p.goal_pose = [3.14/2,0,110] # UAV Goal Position [theta,phi,r] (radians)

p.gcs_tether_l = 0 

print "distance: %f" %p.get_distance()

print "diag distance: %f" %p.get_diagonal_distance()

print "bearing: %f " %p.get_bearing()

ang = p.get_relative_angles()
print "Pose: %.2f, %.2f, %.2f" %(ang[0]*180/np.pi,ang[1]*180/np.pi,ang[2])


p.f_hover = 30

p.run_pose_controller()









