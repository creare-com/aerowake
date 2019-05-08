# -*- coding: utf-8 -*-

"""
This script runs a simple example to see if our calibration allows accurate
stereo reconstruction.  Matching features are manually picked from a pair of
images and fed into simple OpenCV triangulation functions.

From what I can see, the stereo reconstruction seems believable.  The target is
found at a reasonable distance; and while the accuracy isn't stellar, the
detected points are more or less planar (as expected for our flat target).

Note that this script requires the laser metrology toolbox.  This dependency
could be removed without much effort, if necessary.

.. creare_tag::
    :Author: BJD
    :Project: 6598.5
    :Date: 2017/02/10
"""

import os
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2

from laser_metrology_toolbox.point_cloud._functions import best_plane

# Saved calibration file (see stereo_example2.py in laser_metrology_toolbox)
CAL_FILE = 'stereo_ex2_cal.npz'

# Example data to process
DATA_DIR = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                        'Technical Work', 'Testing', 'Data',
                        '20170117 - Stereo Cal Images')
LEFT_FILE = 'Left9.tif'
RIGHT_FILE = 'Right9.tif'

# Load the required data
cal_data = np.load(CAL_FILE)
left_im = plt.imread(os.path.join(DATA_DIR, LEFT_FILE))[:, :, 0]
right_im = plt.imread(os.path.join(DATA_DIR, RIGHT_FILE))[:, :, 0]

# Define the manually matched features
features = [[(923, 761), (857, 712)],   # Left first, Right second
            [(587, 418), (538, 371)],
            [(1292, 398), (1235, 347)],
            [(1336, 1139), (1273, 1094)],
            [(560, 1146), (505, 1090)],
            [(1314, 742), (1252, 693)],
            [(558, 768), (504, 717)],
            [(919, 398), (861, 350)],
            [(934, 1142), (867, 1092)]]

# Display figure illustrating the matched points
vmax = 80
plt.figure('Raw Data')
plt.clf()
ax1 = plt.subplot(121)
ax1.imshow(left_im, cmap='gray', interpolation='nearest', vmax=vmax)
ax1.axis('image')
ax1.set_title('Left Image')
ax2 = plt.subplot(122)
ax2.imshow(right_im, cmap='gray', interpolation='nearest', vmax=vmax)
ax2.axis('image')
ax2.set_title('Right Image')
for pts in features:
    ax1.plot(pts[0][0], pts[0][1], 'o')
    ax2.plot(pts[1][0], pts[1][1], 'o')

# Calculate the 3D points based on the features given
left_points = np.array([pt[0] for pt in features], dtype=float)
right_points = np.array([pt[1] for pt in features], dtype=float)
x1 = cv2.undistortPoints(left_points[None,:,:],
                         cal_data['cameraMatrix1'],
                         cal_data['distCoeffs1'],
                         R=cal_data['R1'], P=cal_data['P1'])[0]
x2 = cv2.undistortPoints(right_points[None,:,:],
                         cal_data['cameraMatrix2'],
                         cal_data['distCoeffs2'],
                         R=cal_data['R2'], P=cal_data['P2'])[0]
points4d = cv2.triangulatePoints(cal_data['P1'], cal_data['P2'], x1.T, x2.T)
points3d = (points4d[:3]/points4d[3, :]).T

# Show the results
fig = plt.figure('3D Plot of Points')
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points3d[:, 0], points3d[:, 1], points3d[:, 2],
           depthshade=False)
ax.set_xlabel('horizontal')
ax.set_ylabel('vertical')
ax.set_zlabel('depth')
ax.set_title('3D Representation of Points from Stereo Reconstruction')
ax.set_xlim(-1500, 1500)
ax.set_ylim(-1500, 1500)
ax.set_zlim(2000, 3000)
scaling = np.array(
    [getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)
ax.view_init(-85, -90)

# Calculate some metrics
plane_normal, target_center, _ = best_plane(points3d)
stand_off = np.linalg.norm(target_center)
deviation = np.dot(plane_normal, (points3d - target_center).T)
rms_error = np.sqrt(np.mean(deviation**2))
print "Results of Stereo on Manually Matched Features"
print "----------------------------------------------"
print " - {0:d} features considered".format(len(points3d))
print " - target is estimated to be {0:0.2f}m from the rig".format(
    stand_off/1000)
print " - RMS deviation from a plane = {0:0.1f}mm".format(rms_error)

plt.show()
