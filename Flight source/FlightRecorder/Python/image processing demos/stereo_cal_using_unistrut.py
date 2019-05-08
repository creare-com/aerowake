# -*- coding: utf-8 -*-

"""
This script runs a calibration of the laboratory stereo system using data
collected from our custom unistrut target.

Note that this script requires the laser metrology toolbox.

Ultimately this script should be moved into a calibration module (inheriting
from the LMT) similar to what we've done for laser scanning projects.

.. creare_tag::
    :Author: BJD
    :Project: Optical Guide II
    :Date: 2017/05/09
"""

import cv2
import copy
import datetime
import itertools
import logging
import numpy as np
import os
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt

from laser_metrology_toolbox.calibration import CameraCalibration

logging.getLogger().setLevel(level=logging.INFO)

# Setup
# CAL_DATE = datetime.date(2017, 1, 17)
# CAL_DATE = datetime.date(2017, 6, 9)
# CAL_DATE = datetime.date(2017, 7, 3)
# CAL_DATE = datetime.date(2017, 7, 27)
CAL_DATE = datetime.date(2018, 1, 26)
show_plots = True                                 # Illustrative plots (slow)
save_file = 'stereo_cal_{}.npz'.format(CAL_DATE)  # Don't save if None

# Where's the data
if CAL_DATE == datetime.date(2017, 1, 17):
    data_dir = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                            'Technical Work', 'Testing', 'Data',
                            '20170117 - Stereo Cal Images')
    file_names_left = ['Left{0:01d}.tif'.format(n) for n in range(10)]
    file_names_right = [fn.replace('Left', 'Right') for fn in file_names_left]
elif CAL_DATE == datetime.date(2017, 6, 9):
    data_dir = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                            'Technical Work', 'Testing', 'Data',
                            '20170609 - Stereo Cal Images')
    file_names_left = ['Left{0:01d}.tif'.format(n) for n in range(10)]
    file_names_right = [fn.replace('Left', 'Right') for fn in file_names_left]
elif CAL_DATE == datetime.date(2017, 7, 3):
    data_dir = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                            'Technical Work', 'Testing', 'Data',
                            '20170703 - Stereo Cal Images')
    file_names_left = ['Left{0:02d}.tif'.format(n) for n in range(10)]
    file_names_right = [fn.replace('Left', 'Right') for fn in file_names_left]
elif CAL_DATE == datetime.date(2017, 7, 27):
    data_dir = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                            'Technical Work', 'Testing', 'Data',
                            '20170727 - LIDAR tour for cal')
    file_names_left = ['leftVisible_000340_051.tif',
                       'leftVisible_000345_362.tif',
                       'leftVisible_000346_671.tif',
                       'leftVisible_000351_732.tif',
                       'leftVisible_000357_817.tif',
                       'leftVisible_000401_531.tif']
    file_names_right = [fn.replace('left', 'right') for fn in file_names_left]
elif CAL_DATE == datetime.date(2018, 1, 26):
    data_dir = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                            'Technical Work', 'Testing', 'Data',
                            '20180126 - Stereo Cal Images')
    # Image 6 crashes OpenCV for some reason
    file_names_left = ['Left{0:d}.tif'.format(n) for n in range(13) if n != 6]
    file_names_right = [fn.replace('Left', 'Right') for fn in file_names_left]

# Parameters for a single camera (define as a dict rather than an ini file)
PARAMS = {'camera': {
              'focal_length_nominal': 3.5,            # mm
              'pixel_size_base': [0.00345, 0.00345],  # mm
              'sensor_size': [2048, 1536],            # pixels
              'working_depth': 2000,                  # mm
              'f_number': 9.5,
              'wavelength': 550},                     # nm
          'calibration_target': {
              'layout': 'irregular',
              'irregular_target_name': 'unistrut_Optical_Guide',
              'feature_style': 'unistrut',
              'unistrut_major_diameter': 28.575,     # mm
              'unistrut_minor_diameter': 14.2875},   # mm
          'calibration': {
              'data_directory': data_dir,
              'camera_threshold_method': 'adaptive',
              'movement_is_precise': False,
              'starting_pos_method': 'manual_match',
              'align_world_to': 'camera',
              'update_working_depth': False,
              'dist_fix_center': True,
              'dist_zero_tangent': True,
              'dist_fix_radial_k': [False, False, False, True, True, True],
              'recalibrate_pix_thresh': [20],
              'params_icci': {'area_ratio': [.25, 3.]}}
          }

# Manual starting points, thresholds and processing parameters
manual_matches_left = os.path.join(data_dir, 'left_matches.ini')
manual_matches_right = os.path.join(data_dir, 'right_matches.ini')
if CAL_DATE == datetime.date(2017, 1, 17):
    thresholds_left = [-15]
    thresholds_right = [-15]
    alpha = .985
    flip_right = False                            # Correct 180 deg in R camera
    stereo_flags = cv2.CALIB_FIX_INTRINSIC        # Use single-camera cals
elif CAL_DATE == datetime.date(2017, 6, 9):
    thresholds_left = [-15]
    thresholds_right = [-15]
    alpha = .83
    flip_right = False
    stereo_flags = cv2.CALIB_FIX_INTRINSIC
elif CAL_DATE == datetime.date(2017, 7, 3):
    thresholds_left = [-15]
    thresholds_right = [-15]
    alpha = 0.
    flip_right = True
    stereo_flags = (cv2.CALIB_USE_INTRINSIC_GUESS +
                    cv2.CALIB_FIX_K3 +
                    cv2.CALIB_FIX_ASPECT_RATIO +   # Force equal focal lengths
                    cv2.CALIB_SAME_FOCAL_LENGTH)   #   helps give reasonable T
elif CAL_DATE == datetime.date(2017, 7, 27):
    thresholds_left = [-29, -32, -28, -26, -30, -30]
    thresholds_right = thresholds_left
    alpha = 0.
    flip_right = False
    stereo_flags = (cv2.CALIB_USE_INTRINSIC_GUESS +
                    cv2.CALIB_FIX_K3 +
                    cv2.CALIB_FIX_ASPECT_RATIO +   # Force equal focal lengths
                    cv2.CALIB_SAME_FOCAL_LENGTH)   #   helps give reasonable T
elif CAL_DATE == datetime.date(2018, 1, 26):
    thresholds_left = [-45]
    thresholds_right = thresholds_left
    alpha = 0.35
    flip_right = True
    stereo_flags = (cv2.CALIB_USE_INTRINSIC_GUESS +
                    cv2.CALIB_ZERO_TANGENT_DIST +
                    cv2.CALIB_FIX_ASPECT_RATIO +   # Force equal focal lengths
                    cv2.CALIB_SAME_FOCAL_LENGTH)   #   helps give reasonable T

## Calibrate left camera

# Do the single-camera calibration on left
print("Calibrating the left camera")
left_params = copy.deepcopy(PARAMS)
left_params['calibration'].update(
    file_names_camera=file_names_left,
    starting_manual_matches=manual_matches_left,
    camera_threshold_values=thresholds_left)
camera_cal_left = CameraCalibration(spec_by_node=left_params)      # Initialize
cal_data_left = camera_cal_left.calibrate(show_report=True,
                                          show_distortion=show_plots,
                                          show_targets=show_plots)

# Things we need for stereo
cameraMatrixL = camera_cal_left.camera_model.camera_matrix
distCoeffsL = camera_cal_left.camera_model.dist_coeffs

## Calibrate right camera

# Do the single-camera calibration on right
print("Calibrating the right camera")
right_params = copy.deepcopy(PARAMS)
right_params['calibration'].update(
    file_names_camera=file_names_right,
    starting_manual_matches=manual_matches_right,
    camera_threshold_values=thresholds_right)
camera_cal_right = CameraCalibration(spec_by_node=right_params)    # Initialize
cal_data_right = camera_cal_right.calibrate(show_report=True,
                                            show_distortion=show_plots,
                                            show_targets=show_plots)

# Things we need for stereo
cameraMatrixR = camera_cal_right.camera_model.camera_matrix
distCoeffsR = camera_cal_right.camera_model.dist_coeffs

plt.show()

## Stereo calibration

# Get the points that are visible to both views
imagePointsL = []
imagePointsR = []
objectPoints = []
for feat_left, feat_right in zip(camera_cal_left.target_feature_graphs,
                                 camera_cal_right.target_feature_graphs):
    lab_and_pos_left = feat_left.labels_and_positions()
    lab_and_pos_right = feat_right.labels_and_positions()
    shared_labels = [lab for lab in lab_and_pos_left
                     if lab in lab_and_pos_right]
    imagePointsL += [np.array([lab_and_pos_left[lab].astype(np.float32)
                               for lab in shared_labels])]
    imagePointsR += [np.array([lab_and_pos_right[lab].astype(np.float32)
                               for lab in shared_labels])]
    objectPoints += [camera_cal_left.cal_target.feature_position(
        shared_labels)[1].astype(np.float32)]
imagePointsR_orig = [pts for pts in imagePointsR]
if flip_right:          # Preprocessing to give effect of flipping image
    imagePointsR = [(np.array(PARAMS['camera']['sensor_size'],
                              dtype=np.float32) - 1) - pts
                    for pts in imagePointsR_orig]

# Do OpenCV stereo calibration
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = \
    cv2.stereoCalibrate(objectPoints, imagePointsL, imagePointsR,
                        cameraMatrixL, distCoeffsL,
                        cameraMatrixR, distCoeffsR,
                        tuple(camera_cal_left.camera_model.image_size),
                        flags=stereo_flags)
print "Total residual from stereo calibration = {0:0.3f}".format(retval)
print 'Left Camera Matrix: '
print cameraMatrix1
print 'Right Camera Matrix: '
print cameraMatrix2
print 'Left Distortion Coefficients'
print distCoeffs1
print 'Right Distortion Coefficients'
print distCoeffs2
print 'R Matrix:'
print R
print 'T Matrix'
print T
print 'Essential Matrix E'
print E
print 'Fundamental Matrix F'
print F

# Check the behavior of F
total_res = 0
for n in range(camera_cal_left.camera_target_models.num_images()):
    x1 = np.hstack([cv2.undistortPoints(imagePointsL[n][None,:,:],
                                        cameraMatrix1,
                                        distCoeffs1,
                                        P=cameraMatrix1)[0],
                    np.ones_like(imagePointsL[n][:,[0]])])
    x2 = np.hstack([cv2.undistortPoints(imagePointsR[n][None,:,:],
                                        cameraMatrix2,
                                        distCoeffs2,
                                        P=cameraMatrix2)[0],
                    np.ones_like(imagePointsL[n][:,[0]])])
    res = (x2 * np.dot(x1, F.T)).sum(axis=1)
    total_res += (res**2).sum()
rms_res = np.sqrt(total_res/np.sum([len(a) for a in imagePointsL]))
print 'RMS agreement of epipolar lines = {0:0.3f} pixels'.format(rms_res)

# Setup the rectified triangulation
#    - Set alpha to avoid field of view issues
#          (see https://github.com/opencv/opencv/issues/7240)
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2,
    tuple(camera_cal_left.camera_model.image_size), R, T, alpha=alpha)

# Save the important calibration parameters
if save_file is not None:
    np.savez(save_file,
             cameraMatrix1=cameraMatrix1, cameraMatrix2=cameraMatrix2,
             distCoeffs1=distCoeffs1, distCoeffs2=distCoeffs2,
             R=R, T=T, E=E, F=F,
             R1=R1, R2=R2, P1=P1, P2=P2, Q=Q)

# Show some verification data using the calibration data
if show_plots:

    # Look at a couple of example epilines
    im_no = 0
    udL = cv2.undistort(cal_data_left.camera_data[im_no],
                        cameraMatrix1, distCoeffs1)
    if flip_right:
        udR = cv2.undistort(cal_data_right.camera_data[im_no][::-1, ::-1],
                            cameraMatrix2, distCoeffs2)
    else:
        udR = cv2.undistort(cal_data_right.camera_data[im_no],
                            cameraMatrix2, distCoeffs2)
    if (CAL_DATE == datetime.date(2017, 1, 17)) and (im_no == 0):
        tmp = np.array([[1125,  744],    # Left0.tif
                        [1125, 1054],
                        [1114,  382],
                        [ 465,  693]], dtype=np.float32)
    elif (CAL_DATE == datetime.date(2017, 6, 9)) and (im_no == 0):
        tmp = np.array([[1020,  802],    # Left0.tif
                        [ 645,  522],
                        [1063,  416],
                        [1398, 1260]], dtype=np.float32)
    elif (CAL_DATE == datetime.date(2017, 7, 3)) and (im_no == 0):
        tmp = np.array([[ 782,  756],    # Left0.tif
                        [1200,  488],
                        [1343, 1172],
                        [1765,  788]], dtype=np.float32)
    elif (CAL_DATE == datetime.date(2017, 7, 27)) and (im_no == 0):
        tmp = np.array([[ 691,  492],    # leftVisible_000340_051.tif
                        [1106,  671],
                        [1528,  863],
                        [1087,  275]], dtype=np.float32)
    elif (CAL_DATE == datetime.date(2018, 1, 26)) and (im_no == 0):
        tmp = np.array([[ 558, 1016],    # Left0.tif
                        [1051,  443],
                        [1667,  837],
                        [1043, 1124]], dtype=np.float32)
    pts = cv2.undistortPoints(tmp[None, :, :], cameraMatrix1,
                              distCoeffs1, P=cameraMatrix1)[0]
    lines = cv2.computeCorrespondEpilines(pts, 1, F)
    plt.figure(num='Example Epi-Lines')
    plt.subplot(121)
    plt.imshow(udL, cmap='gray', interpolation='nearest')
    plt.axis('image')
    plt.plot(pts[:, 0], pts[:, 1], 'rx')
    plt.subplot(122)
    plt.imshow(udR, cmap='gray', interpolation='nearest')
    plt_x = np.array([0, 2048])
    for ln in lines:
        plt_y = (-ln[0, 2] - plt_x*ln[0, 0])/ln[0, 1]
        plt.plot(plt_x, plt_y, 'r-')
    plt.axis('image')

    # Try triangulation
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    scat_colors = itertools.cycle(['r', 'g', 'b', 'y', 'm', 'k'])
    for im_no2 in range(len(imagePointsL)):
        x1 = cv2.undistortPoints(imagePointsL[im_no2][None,:,:],
                                 cameraMatrix1, distCoeffs1,
                                 R=R1, P=P1)[0]
        x2 = cv2.undistortPoints(imagePointsR[im_no2][None,:,:],
                                 cameraMatrix2, distCoeffs2,
                                 R=R2, P=P2)[0]
        points4d = cv2.triangulatePoints(P1, P2, x1.T, x2.T)
        points3d = points4d[:3]/points4d[3, :]
        ax.scatter(points3d[0, :], points3d[1, :], points3d[2, :],
                   c=next(scat_colors), depthshade=False)
    ax.set_xlabel('horizontal')
    ax.set_ylabel('vertical')
    ax.set_zlabel('depth')
    ax.set_xlim(-1500, 1500)
    ax.set_ylim(-1500, 1500)
    ax.set_zlim(500, 4000)
    scaling = np.array(
        [getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)
    ax.view_init(-85, -90)

    # Attempt rectification (tests how appropriate stereoRectify's `alpha` is)
    image_size = tuple(camera_cal_right.camera_model.image_size)
    left_maps = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1,
                                            R1, P1, image_size, cv2.CV_16SC2)
    right_maps = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2,
                                             R2, P2, image_size,
                                             cv2.CV_16SC2)
    if flip_right:   # Need something smarter to handle map2 correctly
        right_maps = (np.array(image_size,
                               dtype=np.int16)[None, None, :] - right_maps[0],
                      right_maps[1])   # Maybe use remap, flip, then remap back
    left_rect = cv2.remap(cal_data_left.camera_data[im_no], *left_maps,
                          interpolation=cv2.INTER_CUBIC)
    right_rect = cv2.remap(cal_data_right.camera_data[im_no], *right_maps,
                           interpolation=cv2.INTER_CUBIC)
    plt.figure(num='Rectified Images')
    plt.subplot(121)
    plt.imshow(left_rect, cmap='gray')
    for y in np.arange(50, 1536, 200):
        plt.plot([0, 2047], [y, y], 'r-')
    plt.axis('image')
    plt.subplot(122)
    plt.imshow(right_rect, cmap='gray')
    for y in np.arange(50, 1536, 200):
        plt.plot([0, 2047], [y, y], 'r-')
    plt.axis('image')

    plt.show()
