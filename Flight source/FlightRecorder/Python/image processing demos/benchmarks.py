# -*- coding: utf-8 -*-
"""
Runs representative OpenCV algorithms for comparison between platforms
"""
print("Importing...")
import cv2
import numpy as np
import scipy as sp
import time
from matplotlib import pyplot as plt
print("Done.")

"""
Compute rectification matrices: cv2.stereoRectify
Prepare undistortion maps for both cameras: cv2.initUndistortRectifyMap
Remap each image: cv2.remap
"""

#Calibration matrices
cameraMatrix1=np.array([[1.05612125e+3, 0e+0, 1.01797316e+3],
                        [0.0e0, 1.05612125e+3, 7.74008007e2],
                        [0.0e0, 0.0e0, 1.0e0]])
cameraMatrix2=np.array([[1.05815970e+3, 0e+0, 1.00633871e+3],
                       [0.0e0, 1.05815970e+3, 7.32887003e+2],
                       [0.0e0, 0.0e0, 1.0e0]])
distCoeffs1=np.array([-0.32260017,0.10400534,0.00083202,0.00041869,-0.01483929,0.0,0.0,0.0])
distCoeffs2=np.array([-0.36100603,0.17792664,0.00062531,0.00078929,-0.05002994,0.0,0.0,0.0])
R=np.array([[ 9.99976550e-1,-4.07412041e-3, 5.50471991e-3],
            [ 4.07838771e-3, 9.99991391e-1,-7.64204731e-4],
            [-5.50155906e-3, 7.86637192e-4, 9.99984557e-1]])
T=np.array([[-160.32546955],[-0.30323083],[-2.59238926]])
imageSize=(2048,1536)

# Prepare stereo information
rectify_scale = 0 # 0=full crop, 1=no crop
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, alpha = rectify_scale)
left_maps  = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, cv2.CV_16SC2)
right_maps = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, cv2.CV_16SC2)

# Relies on a bunch of global variables initialized above.
# Call with "BRISK", "ORB", or "FAST".
def testAlgorithm(alg_string):
    time_spent_processing           = 0
    time_spent_rectifying           = 0
    time_spent_detectingFeatures    = 0
    time_spent_computingDescriptors = 0
    time_spent_matchingFeatures     = 0

    for img_pair in image_list:
        # Load
        left_img  = cv2.imread(base_path + img_pair[0], 0)
        right_img = cv2.imread(base_path + img_pair[1], 0)
        image_size = left_img.shape

        # Rectify
        t0 = time.time()
        left_img_remap  = cv2.remap(left_img,  left_maps [0], left_maps [1], cv2.INTER_LANCZOS4)#, left_img_remap, cv2.BORDER_CONSTANT, 0)
        right_img_remap = cv2.remap(right_img, right_maps[0], right_maps[1], cv2.INTER_LANCZOS4)#LINEAR
        time_spent_rectifying += (time.time() - t0)
        
        # Detect features and compute their descriptors
        t1 = time.time()
        if  (alg_string == 'BRISK'):
            detector = cv2.BRISK_create()
            computer = detector;
        elif(alg_string == 'ORB'):
            detector = cv2.ORB_create()
            computer = detector;
            # Workaround this ORB bug https://github.com/opencv/opencv/issues/6081
            cv2.ocl.setUseOpenCL(False)
        # elif(alg_string == 'FAST'):
            # detector = cv2.FastFeatureDetector()
            # computer = cv2.ORB_create();
            # # Workaround this ORB bug https://github.com/opencv/opencv/issues/6081
            # cv2.ocl.setUseOpenCL(False)
        else:
            raise Exception('Don\'t know algorithm "' + alg_string + '"')
        l_kp      = detector.detect(left_img_remap , None)
        r_kp      = detector.detect(right_img_remap, None)
        time_spent_detectingFeatures += (time.time() - t1)
        t2 = time.time()
        l_kp, l_d = computer.compute(left_img_remap , l_kp)
        r_kp, r_d = computer.compute(right_img_remap, r_kp)
        time_spent_computingDescriptors += (time.time() - t2)
        
        # Match features
        t3 = time.time()
        matcher = cv2.DescriptorMatcher_create("BruteForce")
        matches = matcher.match(l_d, r_d)
        time_spent_matchingFeatures += (time.time() - t3)
        
        time_spent_processing += (time.time() - t0)

    # Compute averages
    time_spent_processing           /= len(image_list)
    time_spent_rectifying           /= len(image_list)
    time_spent_detectingFeatures    /= len(image_list)
    time_spent_computingDescriptors /= len(image_list)
    time_spent_matchingFeatures     /= len(image_list)

    print('For algorithm "' + alg_string + '":')
    print("Average time (in ms) spent on various tasks:")
    print("Processing total: "      + str(time_spent_processing        * 1000.0))
    print("Rectifying: "            + str(time_spent_rectifying        * 1000.0))
    print("Detecting features: "    + str(time_spent_detectingFeatures * 1000.0))
    print("Computing descriptors: " + str(time_spent_computingDescriptors * 1000.0))
    print("Matching features: "     + str(time_spent_matchingFeatures  * 1000.0))
    print('')



base_path = '2017-1-17_stereo_cal_images/'
image_list = [('Left0.tif','Right0.tif'),
              ('Left1.tif','Right1.tif'),
              ('Left2.tif','Right2.tif'),
              ('Left3.tif','Right3.tif'),
              ('Left4.tif','Right4.tif'),
              ('Left5.tif','Right5.tif'),
              ('Left6.tif','Right6.tif'),
              ('Left7.tif','Right7.tif'),
              ('Left8.tif','Right8.tif'),
              ('Left9.tif','Right9.tif'),
]
   
testAlgorithm('BRISK')
testAlgorithm('ORB')
# testAlgorithm('FAST')
