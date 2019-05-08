'''
motion_from_video.py

Created on Mar 30, 2017

@author: cab

Stereo data extraction similar to Fast_orb_tour_images.py

Tested on opencv 2.4.13
'''

import os.path
import subprocess
import glob
import re
from itertools import count, compress
from warnings import warn

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scipy as sp
import numpy as np
import cv2

GIT_COMMIT_HASH = subprocess.check_output(['git','show','--format=%h']).strip()

# Image directory
CAL_FILE = 'stereo_ex2_cal.npz'
 
DATA_DIR = r'\\olympus\Projects\6598-Optical-Guide-II\Technical Work\Testing\Data\20170321 - Stereo tour of Creare\Tour 2 - 1600 - Copy\startup_rec_data'

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Results', 'motion_from_video.py')

# Load calibration data
cal_data = np.load(CAL_FILE)


def image_pair_generator():
    '''
    Load image pairs in order
    
    '''
    warn('Skipping first image because it is poorly exposed.')
    
    for i in count(start=10):  # count(start=20):
        left_filename = os.path.join(DATA_DIR, 'L (%d).tif' % i)
        right_filename = os.path.join(DATA_DIR, 'R (%d).tif' % i)
        
        try:
            left_image = cv2.imread(left_filename, 0)  # Open as grayscale
            right_image = cv2.imread(right_filename, 0)  # Open as grayscale
            
            if left_image.size and right_image.size:
                yield (i, (left_image, right_image))
            else:
                break  # Done when image loading fails
        except:
            break  # Done when image loading fails
    

def display_image_pair(left_img, right_img, title, wait=False):
    
    lr_img = np.concatenate((left_img, right_img), axis=1)
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 1500, 600)
    cv2.imshow(title, lr_img)
    
    if wait:
        cv2.waitKey()


# # Persistent objects for rectify_image_pair
# rectify_scale = 0 # 0=full crop, 1=no crop
# 
# ##if height and width calculated for each image necessary then move in back inside for loop
# (height, width) = [1536, 2048]
# (R1, R2, P1, P2, Q, roi1, roi2) = cv2.stereoRectify(cal_data['cameraMatrix1'], 
#                                                     cal_data['distCoeffs1'], 
#                                                     cal_data['cameraMatrix1'], 
#                                                     cal_data['distCoeffs1'], 
#                                                     (width, height), cal_data['R'], 
#                                                     cal_data['T'], alpha = rectify_scale)

# # Use calibration matrices from calibration file
# cameraMatrix1 = cal_data['cameraMatrix1']
# distCoeffs1 = cal_data['distCoeffs1']
# cameraMatrix2 = cal_data['cameraMatrix2'] 
# distCoeffs2 = cal_data['distCoeffs2'] 
# R = cal_data['R']
# T = cal_data['T']
# R1 = cal_data['R1']
# R2 = cal_data['R2']
# P1 = cal_data['P1']
# P2 = cal_data['P2']


# Persistent objects for find_and_match_features 
detector = cv2.FastFeatureDetector()  # cv2.FastFeatureDetector_create()
descriptor = cv2.ORB()  # cv2.ORB_create()
#matcher = cv2.DescriptorMatcher_create("BruteForce-Hamming")
matcher = cv2.DescriptorMatcher_create("BruteForce")  # Switched from Hamming to support float32 values


def calculate_stereo(left_img, right_img, plot_results=True, display_statistics=True, video_writer=None):
    '''
    calculate_stereo extracted from Fast_orb_tour_images_video_pointcoud_selected images.py [4977bc5]
    
    :param left_img:
    :param right_img:
    '''
    
    if display_statistics:
        print '============= calculate_stereo ============='

        # detect keypoints
    kp1 = detector.detect(left_img)
    kp2 = detector.detect(right_img)
    if display_statistics:
        print '#keypoints in image1: %d, image2: %d' % (len(kp1), len(kp2))

    # compute descriptors
    (kp1, d1) = descriptor.compute(left_img, kp1)
    (kp2, d2) = descriptor.compute(right_img, kp2)
    
    # match the keypoints
    matches = matcher.knnMatch(d1, d2, k=2)
    
    # Apply ratio test
    # Keep only matches in which the first match is significantly closer
    # than the second match
    sel_matches = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            # Removed the brackets around m 
            sel_matches.append(m)

    #  y coordinate masking
    # ======================
    xy1=list()
    xy2=list()
    kp1_idx = []
    kp2_idx = []    
    for m in sel_matches:
        xy1_temp = kp1[m.queryIdx].pt
        xy2_temp = kp2[m.trainIdx].pt
        xy1.append(xy1_temp)
        xy2.append(xy2_temp)
        kp1_idx.append(m.queryIdx)
        kp2_idx.append(m.trainIdx) 

    xy1=np.asarray(xy1).astype(np.float32)
    xy2=np.asarray(xy2).astype(np.float32) 
    kp1_idx = np.array(kp1_idx)
    kp2_idx = np.array(kp2_idx)

    #modified on 03/21/17 
    I = (xy2[:,1]<=1.075*xy1[:,1]) & (xy2[:,1]>=.925*xy1[:,1])
    xy1_filt = xy1[I,:]
    xy2_filt = xy2[I,:]
    kp1_idx = kp1_idx[I]
    kp2_idx = kp2_idx[I]

    # Calculate the 3D points based on the features given

    # left_points1 = np.array([xy1_filt[pt] for pt in range(len(xy1_filt))], dtype=float)
    # right_points1 = np.array([xy2_filt[pt] for pt in range(len(xy2_filt))], dtype=float)
    # disparity=list()
    # for m in xrange(0,len(left_points1)):
    #     diff = (left_points1[m,0] - right_points1[m,0])
    #     disparity.append(diff) 
    disparity = (xy1_filt[:,0] - xy2_filt[:,0])

    # left_points = [];right_points = []
    # for i in xrange(len(disparity)):
    #     if disparity[i] > 3:
    #         # Removed the brackets around m 
    #         left_points.append(left_points1[i,:])
    #         right_points.append(right_points1[i,:])
    # left_points = np.array([left_points[pt] for pt in range(len(left_points))], dtype=float)
    # right_points = np.array([right_points[pt] for pt in range(len(right_points))], dtype=float)
    I = (disparity > 3.0)
    
    left_points = xy1_filt[I,:]
    right_points = xy2_filt[I,:]
    kp1_idx = kp1_idx[I]
    kp2_idx = kp2_idx[I]
         
    if left_points.size:  # is not empty
        x11 = cv2.undistortPoints(left_points[None,:,:],cal_data['cameraMatrix1'],
                             cal_data['distCoeffs1'],
                             R=cal_data['R1'], P=cal_data['P1'])[0]
        x2 = cv2.undistortPoints(right_points[None,:,:],
                             cal_data['cameraMatrix2'],
                             cal_data['distCoeffs2'],
                             R=cal_data['R2'], P=cal_data['P2'])[0]
    else:
        x11 = np.empty(shape=(0, 2), dtype=np.float32)
        x2 = np.empty(shape=(0, 2), dtype=np.float32)
    
    #Masking x,y space, might consider removing, modifying, or testing for robustness, 
    #might consider masking some percentage of the image borders...
    # x1 = x11[(x11[:,0]>250)&(x11[:,0]<2000)]
    # x2 = x2[(x11[:,0]>250)&(x11[:,0]<2000)]
    # x2 = x2[(x1[:,1]>400)&(x1[:,1]<1500)]  
    # x1 = x1[(x1[:,1]>400)&(x1[:,1]<1500)]
    I = ((x11[:,0]>250) & (x11[:,0]<2000) & (x11[:,1]>400) & (x11[:,1]<1500))
    x1 = x11[I,:]
    x2 = x2[I,:]
    kp1_idx = kp1_idx[I]
    kp2_idx = kp2_idx[I]    

    if np.any(I):
        points4d = cv2.triangulatePoints(cal_data['P1'], cal_data['P2'], x1.T, x2.T)
        points3d = (points4d[:3]/points4d[3, :]).T
    else:
        points3d = np.empty(shape=(0, 3), dtype=np.float32)
        
    points3d_filt=points3d #[(points3d[:,2]>=0) & (points3d[:,2]<50000)]#


    if plot_results:
        #data=plot_functions.video_subplot(left_img,x1,x2,points3d_filt,ii,1) 
        vmax = 80
        fig=plt.figure(num=2, figsize=(12, 5.5))
        plt.clf()
        ax1 = plt.subplot(121)
        ax1.imshow(left_img, cmap='gray', interpolation='nearest', vmax=vmax)
        ax1.axis('image')
        ax1.set_title('Left Image')
        ax1.plot(x1[:,0], x1[:,1], '.',x2[:,0], x2[:,1], '.')
        leg = plt.legend(['Left', 'Right'],frameon=False, loc = 'best')
        for text in leg.get_texts():
            plt.setp(text, color = 'w')
        
        ax1 = fig.add_subplot(122, projection='3d')
        ax1.scatter(points3d_filt[:,0], points3d_filt[:,2], -points3d_filt[:,1], depthshade=False)
        ax1.set_xlabel('Horizontal')
        ax1.set_ylabel('Depth')
        ax1.set_zlabel('Vertical')
        ax1.set_title('3D Representation of Points from Stereo Reconstruction')
        ax1.set_xlim(-1500, 1500)
        ax1.set_ylim(1000, 4000)
        ax1.set_zlim(-1500, 1500)
        ax1.view_init(12.5, -60)
        fig.tight_layout()
        fig.canvas.draw()
        
            # Now we can save it to a numpy array.
        data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        #print data.shape
        #print data.reshape(fig.canvas.get_width_height()[::-1]+ (3,))
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))  
        
        if video_writer is not None: 
            video_writer.write(data)
    
    
    if display_statistics:
        # Show the results
    
        print "----------------------------------------------"     
        #post process and outputs
        print '#keypoints in image1: %d, image2: %d' % (len(kp1), len(kp2))
        print '#matches:', len(matches)
        print '#selected matches:', len(sel_matches)
        print '#filtered matches:', len(xy1_filt)
        print '#filtered triangulated:', len(points3d_filt[:,2])
        if points3d_filt.size:
            print 'average depth in mm is', sum(points3d_filt[:,2])/len(points3d_filt[:,2])   
        
            target_center = np.mean(points3d_filt, axis=0)
            # Calculate the plane normal
            tmp = points3d - target_center
            [D, V] = np.linalg.eig(np.dot(tmp.T, tmp))
            V = V  # match MATLAB.
            I = np.argsort(np.abs(D))
            plane_normal = V[:, I[0]]
            n2 = V[:, I[-1]]
        
            stand_off = np.linalg.norm(target_center)
            deviation = np.dot(plane_normal, (points3d_filt - target_center).T)
            rms_error = np.sqrt(np.mean(deviation**2))
            
            print "Results of Stereo on Manually Matched Features"
            print "----------------------------------------------"
            print " - {0:d} features considered".format(len(points3d_filt))
            print " - target is estimated to be {0:0.2f}m from the rig".format(stand_off/1000)
            print " - RMS deviation from a plane = {0:0.1f}mm".format(rms_error)        


    # =======================================================
    #    Assemble data used by motion from video
    
    # Filter points based on Z-coordinate
    I = ((points3d_filt[:,2] >= 100) & (points3d_filt[:,2] < 20000))
    points3d_filt = points3d_filt[I, :]
    kp1_idx = kp1_idx[I]
    kp2_idx = kp2_idx[I]      
    
    # Distorted image coordinates
    kp_left_uv = np.array([kp.pt for kp in kp1])
    kp_right_uv = np.array([kp.pt for kp in kp2])
    
    # Rectified image coordinates
    kp_left_xy = cv2.undistortPoints(kp_left_uv[np.newaxis,:,:],
        cal_data['cameraMatrix1'], cal_data['distCoeffs1'],
        R=cal_data['R1'], P=cal_data['P1'])[0]
    kp_right_xy = cv2.undistortPoints(kp_right_uv[np.newaxis,:,:],
        cal_data['cameraMatrix2'], cal_data['distCoeffs2'],
        R=cal_data['R2'], P=cal_data['P2'])[0]

    # Descriptors
    kp_left_d = d1
    kp_right_d = d2
    
    # 3D points and corresponding descriptors in left and right images
    matched_points3d = points3d_filt
    # matched_xy_left = kp_left_xy[kp1_idx,:]
    # matched_xy_right = kp_right_xy[kp2_idx,:]
    # matched_d_left = d1[kp1_idx,:]
    # matched_d_right = d2[kp2_idx,:]
    
    # Calculate 3D points which exactly project to image points
    x1 = kp_left_xy[kp1_idx,:]
    x2 = kp_right_xy[kp2_idx,:]
    
    matched_points3d_left = constrainedTriangulatePoints(cal_data['P1'], cal_data['P2'], x1.T, x2.T).T
    matched_points3d_right = constrainedTriangulatePoints(cal_data['P2'], cal_data['P1'], x2.T, x1.T).T
    

    mfv_data = {'kp_left':  {'uv': kp_left_uv,  # [n_left_keypoints x 2] ndarray of distorted image coordinates
                             'xy': kp_left_xy,  # [n_left_keypoints x 2] ndarray of rectified image coordinates
                             'd':  kp_left_d},  # [n_left_keypoints x n_descriptors] ndarray of descriptors
                'kp_right': {'uv': kp_right_uv, # [n_right_keypoints x 2] ndarray of distorted image coordinates 
                             'xy': kp_right_xy, # [n_right_keypoints x 2] ndarray of rectified image coordinates
                             'd':  kp_right_d}, # [n_right_keypoints x n_descriptors] ndarray of descriptors
                'matched':  {'points3d': matched_points3d, # [n_matched x 3] ndarray of 3d point coordinates
                             'points3d_left': matched_points3d_left, # [n_matched x 3] ndarray of 3d point coordinates which project to left image points
                             'points3d_right': matched_points3d_right, # [n_matched x 3] ndarray of 3d point coordinates which project to right image points
                             'kp_idx_left':  kp1_idx,      # [n_matched] ndarray of keypoint indices
                             'kp_idx_right': kp2_idx},     # [n_matched] ndarray of keypoint indices
                }

    return (points3d_filt, mfv_data)

    
# Motion from video (MFV) descriptor matcher    
matcher_mfv = cv2.DescriptorMatcher_create("BruteForce-Hamming") 

# matches_1 = match_descriptors([d1p_filt, d2p_filt], d1)
def match_descriptors(queryDescriptorsList, trainDescriptors):
    '''
    Find the closest trainDescriptors element corresponding to each
    the elements in queryDescriptors
    
    :param queryDescriptorsList: list of m ndarray objects, each of size (np, nd)
    :param trainDescriptors: ndarray of size (nf, nd)

    m : number of descriptors for each 3D point
    nf : number of features
    np : number of 3D points
    nd : dimension of descriptors
    '''
    
    # Note: k = 2  # Number of nearest neighbors
    
    # Calculate the matches for each queryDescriptors
    #matches_list = [matcher_mfv.match(qd, trainDescriptors) for qd in queryDescriptorsList]
    matches_list = [matcher_mfv.knnMatch(qd, trainDescriptors, k=2) for qd in queryDescriptorsList]
    # Note: len(matches_list) = m, len(matches_list[i]) = np, len(matches_list[i][j]) = k 
    # TODO: Make more efficient. (Should only train descriptors once.)

    # "Transpose" data, so that outer index corresponds to a 3D point
    matches_pairs = zip(*matches_list)
    # Note: len(matches_pairs) = np, len(matches_pairs[i]) = m, len(matches_pairs[i][j]) = k 
    
    # For each 3D point, select the match with the lowest distance
    best_matches = [min(enumerate(mp), key=lambda (i,x):x[0].distance) for mp in matches_pairs]
    # Note: len(best_matches) = np
    #   best_matches[j] = (i, mp)
    #        where i is the index of queryDescriptorsList
    #        and mp is a tuple of the best two matches in queryDescriptorsList[i]
    
    # Keep only matches with high confidence
    confident_matches = [(i, m1) for (i, (m1, m2)) in best_matches if m1.distance < 0.75*m2.distance]
    
    return confident_matches


def constrainedTriangulatePoints(P1, P2, x1, x2):
    '''
    Find 3-D points that *exactly* project to rectified points in camera 1 image
    
    Note: see derivation in CAB Optical Guide notebook, 5/2/2017
    
    :param P1: projection matrix for camera 1 (P1.shape = (3, 4))
    :param P2: projection matrix for camera 2 (P2.shape = (3, 4))
    :param x1: rectified points in camera 1 image  (x1.shape = (2, N))
    :param x2: rectified points in camera 2 image  (x2.shape = (2, N))
    '''

    assert P1[0,0] == P1[1,1], "P1 x and y axes must have same focal length"
    assert P2[0,0] == P2[1,1], "P2 x and y axes must have same focal length"
    assert P1[0,0] == P2[0,0], "P1 and P2 must have equal focal lengths"
    
    # Extract submatrices
    f = P1[0,0]
    c1 = P1[0:2,[2]]
    t1 = P1[0:2,[3]]
    
    c2 = P2[0:2,[2]]
    t2 = P2[0:2,[3]]
    
    c21 = c2 - c1  # Difference in image centers
    t21 = t2 - t1  # Difference in pinhole centers
    
    d = x2 - x1    # Calculate disparities
    
    A = d - c21    # Subtract difference of image centers (broadcast across columns)
    
    # Original version (includes factor of w in term to be minimized)
    # invAtA = 1. / np.sum(np.square(A), axis=0)
    # Atb = np.sum(A * t21, axis=0)
    #  
    # zhat = (invAtA * Atb)[np.newaxis,:]  # Estimated z coordinates
    # xyhat = ((x1 - c1)*zhat - t1) / f    # Project along ray to find x and y coordinates
    # xyzhat = np.concatenate((xyhat, zhat),axis=0)  
    
    # Modified version (should be more accurate)
    btb = np.sum(np.square(t21), axis=0)[np.newaxis,:]
    btA = np.sum(t21 * A, axis=0)[np.newaxis,:]
    
    zhat1 = btb/btA  # Estimated z coordinates
    xyhat1 = ((x1 - c1)*zhat1 - t1) / f    # Project along ray to find x and y coordinates
    xyzhat1 = np.concatenate((xyhat1, zhat1), axis=0)  
    
    return xyzhat1
    

def solve_motion(kp_data, matched_data, matches, camera, points_before_image):
    '''
    
    :param kp_data: keypoint data for a single image
    :param matched_data: 3d data for matched keypoints in a stereo pair
    :param matches: list of matches
    :param camera: 'left' or 'right' (identifying sd_image)
    :param points_before_image: True or False
    '''

    if camera == 'left':
        cameraMatrix = cal_data['cameraMatrix1']
        distCoeffs = cal_data['distCoeffs1']
        Rs = cal_data['R1']
        Ps = cal_data['P1']
    elif camera == 'right':
        cameraMatrix = cal_data['cameraMatrix2']
        distCoeffs = cal_data['distCoeffs2']
        Rs = cal_data['R2']
        Ps = cal_data['P2']
    else:
        raise RuntimeError("camera must be left or right")
    
    # Compile list of distorted image coordinates of matched keypoints
    im_pts = kp_data['uv'][[m.trainIdx for (i, m) in matches],:]
    
    # Compile list of 3D coordinates of matched keypoints
    points3d = np.concatenate((matched_data['points3d_left'][:,:,np.newaxis],
                               matched_data['points3d_right'][:,:,np.newaxis]),
                              axis=2)
    
    # Select 3D coordinates of matched descriptor from matched image
    pc_pts_list = [points3d[[m.queryIdx],:,i] for (i,m) in matches]
    pc_pts = np.concatenate(pc_pts_list, axis=0)  # Aggregate into n x 3 numpy array
    
    # Fit image points to point cloud
    (rvec_pnp, tvec_pnp, inliers) = cv2.solvePnPRansac(pc_pts, im_pts,
        cameraMatrix=cameraMatrix, distCoeffs=distCoeffs, )   # minInliersCount
    
    # Calculate rotation and translation of camera relative to previous camera position
    R_pnp = cv2.Rodrigues(rvec_pnp)[0]
    
    # Extract stereo offset from Ps
    ts = np.linalg.solve(Ps[:,0:3], Ps[:,[3]])
    
    Rc = np.dot(R_pnp.T, Rs)
    tc = np.dot(R_pnp.T, (np.dot(Rs, ts) - tvec_pnp))
    
    if points_before_image:
        rvec = cv2.Rodrigues(Rc)[0]
        tvec = tc
    else:
        # Invert transformation
        rvec = cv2.Rodrigues(Rc.T)[0]
        tvec = np.dot(Rc.T, -1*tc)
    
    # Store data for plotting
    solve_motion_data = locals()
    
    return (rvec, tvec, solve_motion_data)


def motion_from_video(mfv_data, mfv_data_prev, display_statistics=True):
    '''
    Calculate change in translation and rotation relative to previous pose using
    point data
    
    :param mfv_data:
    :param mfv_data_prev:
    '''
    
    if display_statistics:
        print '============= motion_from_video ============='
        
    # Extract keypoint descriptors
    t0_left_d = mfv_data_prev['kp_left']['d']
    t0_right_d = mfv_data_prev['kp_right']['d']
    t1_left_d = mfv_data['kp_left']['d']
    t1_right_d = mfv_data['kp_right']['d']
    
    # Extract restricted set of descriptors which have valid 3D points
    t0_3d_d_left = mfv_data_prev['kp_left']['d'][mfv_data_prev['matched']['kp_idx_left'],:]
    t0_3d_d_right = mfv_data_prev['kp_right']['d'][mfv_data_prev['matched']['kp_idx_right'],:]
    t1_3d_d_left = mfv_data['kp_left']['d'][mfv_data['matched']['kp_idx_left'],:]
    t1_3d_d_right = mfv_data['kp_right']['d'][mfv_data['matched']['kp_idx_right'],:]
    
    # Match descriptors with 3D data to descriptors from previous/next time point
    matches_0_left = match_descriptors([t1_3d_d_left, t1_3d_d_right], t0_left_d)
    matches_0_right = match_descriptors([t1_3d_d_left, t1_3d_d_right], t0_right_d)
    matches_1_left = match_descriptors([t0_3d_d_left, t0_3d_d_right], t1_left_d)
    matches_1_right = match_descriptors([t0_3d_d_left, t0_3d_d_right], t1_right_d)
    
    # Build a list of all match distances
    dist = [m.distance for (i, m) in (matches_1_left + matches_1_right + matches_0_left + matches_0_right)]
    
    if dist:  # is not empty
        # Distance threshold: 50th percentile of all distances
        thres_dist = np.percentile(np.array(dist), 50)
    
        # Discard matches over threshold
        matches_0_left =  [(i, m) for (i, m) in matches_0_left  if m.distance <= thres_dist]
        matches_0_right = [(i, m) for (i, m) in matches_0_right if m.distance <= thres_dist]
        matches_1_left =  [(i, m) for (i, m) in matches_1_left  if m.distance <= thres_dist]
        matches_1_right = [(i, m) for (i, m) in matches_1_right if m.distance <= thres_dist]

    # Select approach with most accepted matches
    num_matches = [len(ml) for ml in [matches_1_left, matches_1_right, matches_0_left, matches_0_right]]
    max_matches = max(num_matches)

    if display_statistics:
        print "#selected matches: %s" % (num_matches, )

    # If best option has at least 10 matches
    if max_matches >= 10:
        
        if len(matches_1_left) == max_matches:
            # Match image 1 key points to previous point cloud
            (rvec, tvec, solve_motion_data) = solve_motion(kp_data=mfv_data['kp_left'],
                                        matched_data=mfv_data_prev['matched'],
                                        matches=matches_1_left,
                                        camera='left',
                                        points_before_image=True)
            
        elif len(matches_1_right) == max_matches:
            # Match image 2 key points to previous point cloud
            (rvec, tvec, solve_motion_data) = solve_motion(kp_data=mfv_data['kp_right'],
                                        matched_data=mfv_data_prev['matched'],
                                        matches=matches_1_right,
                                        camera='right',
                                        points_before_image=True)
            
        elif len(matches_0_left) == max_matches:
            # Match previous image 1 key points to current point cloud
            (rvec, tvec, solve_motion_data) = solve_motion(kp_data=mfv_data_prev['kp_left'],
                                        matched_data=mfv_data['matched'],
                                        matches=matches_0_left,
                                        camera='left',
                                        points_before_image=False)
            
        elif len(matches_0_right) == max_matches:
            # Match previous image 2 key points to current point cloud
            (rvec, tvec, solve_motion_data) = solve_motion(kp_data=mfv_data_prev['kp_right'],
                                        matched_data=mfv_data['matched'],
                                        matches=matches_0_right,
                                        camera='right',
                                        points_before_image=False)
            
    else:
        # Too few points available to make a match
        rvec = None
        tvec = None
        solve_motion_data = None
    
    if display_statistics:
        print "rvec: %s" % (rvec, )
        print "tvec: %s" % (tvec, )
        
    return (rvec, tvec, solve_motion_data)


def undistort_image_pair(image_pair):
    (left_img_orig, right_img_orig) = image_pair
    
    left_img = cv2.undistort(left_img_orig,
                    cal_data['cameraMatrix1'], cal_data['distCoeffs1'])
    right_img = cv2.undistort(right_img_orig,
                    cal_data['cameraMatrix2'], cal_data['distCoeffs2'])
    
    return (left_img, right_img)


def plot_motion_from_video(rvec, tvec, image_pair, image_pair_prev, solve_motion_data, filename=None):
    
    def plot_images():
        # Undistort images
        rect_image_pair = undistort_image_pair(image_pair)
        rect_image_pair_prev = undistort_image_pair(image_pair_prev)
        
        # Plot images
        def plot_image_ax(ax_i, im_i, title=''):
            vmax=255
            
            ax_i.imshow(im_i, cmap='gray', interpolation='nearest', vmax=vmax)
            ax_i.axis('image')
            ax_i.set_title(title)
            ax_i.get_xaxis().set_visible(False)
            ax_i.get_yaxis().set_visible(False)
            # ax_i.plot()
        
        plot_image_ax(ax[0,0], rect_image_pair_prev[0], 'Previous Left Image')
        plot_image_ax(ax[0,1], rect_image_pair_prev[1], 'Previous Right Image')
        plot_image_ax(ax[1,0], rect_image_pair[0], 'New Left Image')
        plot_image_ax(ax[1,1], rect_image_pair[1], 'New Right Image')    
    

    def project_points(P, p_xyz):
        # Project points onto *undistorted* image (note: not rotated image)
         
        p_uvw = np.dot(P[:,0:3], p_xyz) + P[:,[3]]  # Project into image (3D image coordinates)
        p_uv = p_uvw[0:2,:] / p_uvw[[2],:]  # Divide by depth to get 2D image coordinates
         
        return p_uv
        
    # Plot matches
    def plot_matches():
        if solve_motion_data is not None:
            ax_colors = [['b','g'],['r','c']]
            
            matches = solve_motion_data['matches']
            if solve_motion_data['points_before_image']:
                points_ax = ax[0,:]
                points_colors = ax_colors[0]
                image_ax = ax[1,:]
                image_colors = ax_colors[1]
            else:
                points_ax = ax[1,:]
                points_colors = ax_colors[1]
                image_ax = ax[0,:]
                image_colors = ax_colors[0]
        
            if solve_motion_data['camera'] == 'left':
                image_ax = image_ax[0]
                image_colors = image_colors[0]
            else:
                image_ax = image_ax[1]
                image_colors = image_colors[1]
        
            # List of distorted image coordinates
            im_pts = solve_motion_data['im_pts']
        
             # List of 3D coordinates of matched keypoints
            points3d = solve_motion_data['points3d']
            
            I_left = np.array([i == 0 for (i,m) in matches])
            I_right = np.logical_not(I_left)
            #pc_pts_list_left = [points3d[[m.queryIdx],:,i] for (i,m) in matches if i == 0]
            #pc_pts_list_right = [points3d[[m.queryIdx],:,i] for (i,m) in matches if i == 1]
            
            pc_pts = solve_motion_data['pc_pts']
            pc_pts_left = pc_pts[I_left,:].T
            pc_pts_right = pc_pts[I_right,:].T
            
            # Undistort (but don't rotate) im_pts
            im_pts_uv = cv2.undistortPoints(im_pts[np.newaxis,:,:],
                solve_motion_data['cameraMatrix'], solve_motion_data['distCoeffs'],
                P=solve_motion_data['cameraMatrix'])[0].T
            im_pts_left_uv = im_pts_uv[:,I_left]
            im_pts_right_uv = im_pts_uv[:,I_right]
            
            # Project pc_pts
            def calculate_P_eff(Pc, Rc, camera_matrix):
                return np.dot(camera_matrix, np.linalg.solve(np.dot(Pc[:,0:3], Rc), Pc))
            
            P_eff_left = calculate_P_eff(cal_data['P1'], cal_data['R1'], cal_data['cameraMatrix1'])
            pc_pts_left_uv = project_points(P_eff_left, pc_pts_left)
            
            P_eff_right = calculate_P_eff(cal_data['P2'], cal_data['R2'], cal_data['cameraMatrix2'])
            pc_pts_right_uv = project_points(P_eff_right, pc_pts_right)
            
            # Plot points on images
            image_ax.plot(im_pts_uv[0,I_left], im_pts_uv[1,I_left], points_colors[0] + 'o')
            image_ax.plot(im_pts_uv[0,I_right], im_pts_uv[1,I_right], points_colors[1] + 'o')
            image_ax.plot(im_pts_uv[0,:], im_pts_uv[1,:], image_colors + '.')
            
            for i in range(pc_pts_left_uv.shape[1]):
                points_ax[0].plot([pc_pts_left_uv[0,i], im_pts_left_uv[0,i]],
                                  [pc_pts_left_uv[1,i], im_pts_left_uv[1,i]],
                                  image_colors + '-')
    
            for i in range(pc_pts_right_uv.shape[1]):
                points_ax[1].plot([pc_pts_right_uv[0,i], im_pts_right_uv[0,i]],
                                  [pc_pts_right_uv[1,i], im_pts_right_uv[1,i]],
                                  image_colors + '-')
            
            
            points_ax[0].plot(pc_pts_left_uv[0,:], pc_pts_left_uv[1,:], points_colors[0] + '.')
            points_ax[1].plot(pc_pts_right_uv[0,:], pc_pts_right_uv[1,:], points_colors[1] + '.')
    
    def plot_trails():
        if rvec is not None:
            # Plot trails
            delta_xy = 100.0  # mm
            delta_z = 1000.0  # mm
            p_xyz = np.array([[delta_xy, delta_xy, delta_z],
                          [delta_xy, -1.0*delta_xy, delta_z],
                          [-1.0*delta_xy, -1.0*delta_xy, delta_z],
                          [-1.0*delta_xy, delta_xy, delta_z],
                          ]).T
         
            R = cv2.Rodrigues(rvec)[0]
            p0_xyz0 = p_xyz  # Start of trails in initial camera reference frame
            p1_xyz1 = p_xyz  # End of trails in final camera reference frame 
            p0_xyz1 = np.dot(R.T, p0_xyz0 - tvec)  # Start of trails in final camera reference frame
            p1_xyz0 = np.dot(R, p1_xyz1) + tvec  # End of trails in initial camera reference frame
             
            def plot_trails_ax(ax_i, p0, p1, Pc, Rc, camera_matrix):
                # Plot trails on *undistorted* image (note: not rotated image)
                 
                P_eff = np.dot(camera_matrix, np.linalg.solve(np.dot(Pc[:,0:3], Rc), Pc))
                 
                p0_uv = project_points(P_eff, p0)
                p1_uv = project_points(P_eff, p1)
                 
                for i in xrange(p0_uv.shape[1]):
                    # ax_i.plot([p0_uv[0,i], p1_uv[0,i]], [p0_uv[1,i], p1_uv[1,i]], 'm-')
                    # ax_i.plot([p1_uv[0,i]], [p1_uv[1,i]], 'mo')
                    ax_i.arrow(p0_uv[0,i], p0_uv[1,i],
                               p1_uv[0,i] - p0_uv[0,i], p1_uv[1,i] - p0_uv[1,i],
                               width=10.0, color='m')
                 
            plot_trails_ax(ax[0,0], p0_xyz0, p1_xyz0, cal_data['P1'], cal_data['R1'], cal_data['cameraMatrix1'])
            plot_trails_ax(ax[0,1], p0_xyz0, p1_xyz0, cal_data['P2'], cal_data['R2'], cal_data['cameraMatrix2'])
            plot_trails_ax(ax[1,0], p0_xyz1, p1_xyz1, cal_data['P1'], cal_data['R1'], cal_data['cameraMatrix1'])
            plot_trails_ax(ax[1,1], p0_xyz1, p1_xyz1, cal_data['P2'], cal_data['R2'], cal_data['cameraMatrix2'])    



    fig = plt.figure(num=3)
    plt.close(fig)
    (fig, ax) = plt.subplots(nrows=2, ncols=2, figsize=(12.0, 9.0), num=3)
    
    plot_images()
    plot_matches()
    plot_trails()
    
    fig.show()
    fig.canvas.draw()
    fig.canvas.flush_events()
    
    if filename is not None:
        fig.savefig(filename)
    
                       
    # raise RuntimeError('============= UNDER CONSTRUCTION ==============')
    
    return


def main():

    mfv_data = None
    
    if OUTPUT_DIR is not None and not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
    
    # Loop through all images
    for (i, image_pair) in image_pair_generator():

        (left_image, right_image) = image_pair
        
        # Cache previous motion from video data
        mfv_data_prev = mfv_data

        # Display original image pair 
        display_image_pair(left_image, right_image, 'Original', wait=False)
        
        # Calculate stereo data
        (_, mfv_data) = calculate_stereo(left_image, right_image)
        
        # Calculate motion from video
        # warn('**** DEBUGGING: SAME INPUT AND OUTPUT IMAGE ****')
        # (rvec, tvec, solve_motion_data) = motion_from_video(mfv_data, mfv_data)
    
        if mfv_data_prev is not None:
            (rvec, tvec, solve_motion_data) = motion_from_video(mfv_data, mfv_data_prev)

            # Plot motion from video results
            if OUTPUT_DIR is not None:
                output_filename = os.path.join(OUTPUT_DIR, 'mfv-%04i.png' % i)
            else:
                output_filename = None
                
            plot_motion_from_video(rvec, tvec, image_pair, image_pair_prev,
                                   solve_motion_data, filename=output_filename)
        
            #raise RuntimeError('============= UNDER CONSTRUCTION ==============')
            if False:
                plt.show()
                #cv2.waitKey()
                
            pass
        
    
        image_pair_prev = image_pair
    pass



def main_stereo_test():
    
    # Loop through all images
    for (i, image_pair) in image_pair_generator():

        (left_image, right_image) = image_pair
        
        # Display original image pair 
        display_image_pair(left_image, right_image, 'Original', wait=False)
        
        # Calculate stereo
        (_, mfv_data) = calculate_stereo(left_image, right_image)
        
    pass    


if __name__ == "__main__":
    main()
    # main_stereo_test()
