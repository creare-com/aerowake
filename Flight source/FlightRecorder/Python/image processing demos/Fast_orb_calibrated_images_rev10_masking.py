# -*- coding: utf-8 -*-
"""
Created on Mon Jan 16 10:54:27 2017

@author: fiv
"""

"""
Created on Thu Sep 29 16:12:57 2016
@author: fiv
In this revision we will clean up the code, features are found in distorted space, matched.  
Matched features are then undistorted and triangulated.
This is a cleaned up version from rev6_manually.  The other difference is that based on an independent
study we can get almost 50% more data points if we remove the distance filter and filter only on 
vertical allignment.  We will see how well/worse this does to the point cloud generation in this version
"""

import cv2
import os
import scipy as sp
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import itertools

"""define image pair"""
#6755 left, 6575 right
CAL_FILE = 'stereo_ex2_cal.npz'

#DATA_DIR = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
 #                       'Technical Work', 'Testing', 'Data',
  #                      '20170117 - Stereo Cal Images')
DATA_DIR1 = os.path.join(r'C:\Users','fiv','Desktop','optical guide','dist_calibration')
print DATA_DIR1

LEFT_FILE = 'L6.TIF'
RIGHT_FILE = 'R6.TIF'

#DATA_DIR = os.path.join('\\\\C', 'fiv', 'Documents',
                        #'Python Scripts', 'depth map', 'point cloud trials',
                        #'matching performance')
# Load the required data
cal_data = np.load(CAL_FILE)
print os.path.join(DATA_DIR1, LEFT_FILE)
mask_left = plt.imread(os.path.join(DATA_DIR1, LEFT_FILE))[:, :, 0]
mask_right = plt.imread(os.path.join(DATA_DIR1, RIGHT_FILE))[:, :, 0]

[height,width]= mask_left.shape
left_img = np.zeros(mask_left.shape,np.uint8)
right_img = np.zeros(mask_left.shape,np.uint8)
per=0.1
left_img[per*height:(1-per)*height,(per)*width:(1-per)*width] = mask_left[per*height:(1-per)*height,(per)*width:(1-per)*width]
right_img[per*height:(1-per)*height,(per)*width:(1-per)*width] = mask_right[per*height:(1-per)*height,(per)*width:(1-per)*width]


original = np.concatenate((left_img, right_img), axis=1)
cv2.namedWindow('original',cv2.WINDOW_NORMAL)
cv2.resizeWindow('original', 1500,600)
cv2.imshow("original", original)
cv2.waitKey(0)

print_file=0 #if selected matches image desired
point_cloud=0 #if point cloud file desired (format x,y,z), where z is depth
thicker_line=1

#feature detection functions defined
detector = cv2.FastFeatureDetector_create()
cv2.ocl.setUseOpenCL(False)
descriptor = cv2.ORB_create()
# create BFMatcher object
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
#matcher = cv2.DescriptorMatcher_create("BruteForce-Hamming")



rectify_scale = 0 # 0=full crop, 1=no crop
R=np.array([[9.9997655e-1,-4.07412041e-3,5.50471991e-3],
   [4.07838771e-3,9.99991391e-1,-7.64204731e-4],
    [-5.50155906e-3,7.86637192e-4,9.99984557e-1]])
T=np.array([[-160.32546955],[-0.30323083],[-2.59238926]])

R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(cal_data['cameraMatrix1'], 
                                                  cal_data['distCoeffs1'], 
                                                  cal_data['cameraMatrix1'], 
                                                  cal_data['distCoeffs1'], 
                                                  (width, height), cal_data['R'], 
                                                    cal_data['T'], alpha = rectify_scale)
# detect keypoints
kp1 = detector.detect(left_img)
kp2 = detector.detect(right_img)
(kp1, d1) = descriptor.compute(left_img, kp1)
(kp2, d2) = descriptor.compute(right_img, kp2)
print '#keypoints in image1: %d, image2: %d' % (len(kp1), len(kp2))

# match the keypoints
matches = matcher.match(d1, d2)
# threshold: half the mean
dist = [m.distance for m in matches]  
print 'Min, Max and Mean distance are: %d, %d, %d' % (min(dist), max(dist), (sum(dist)/len(dist)))
thres_dist = (sum(dist) / len(dist)) * 0.5

# keep only the reasonable matches, the lower the value of dist the better
sel_matches =[m for m in matches if m.distance > 3.5]#matches#

'''y coordinate masking'''
xy1=list()
xy2=list()
for m in sel_matches:
    xy1_temp = kp1[m.queryIdx].pt
    xy2_temp = kp2[m.trainIdx].pt
    xy1.append(xy1_temp)
    xy2.append(xy2_temp)

xy1=np.asarray(xy1).astype(np.float32)
xy2=np.asarray(xy2).astype(np.float32)

#xy_sel1 =xy1
#xy_sel2 =xy2
       
vmax = 80
fig=plt.figure('Left Image')
plt.clf()
ax1 = plt.subplot(111)
ax1.imshow(left_img, cmap='gray', interpolation='nearest', vmax=vmax)
ax1.axis('image')
ax1.set_title('Left Image')
ax1.plot(xy1[:,0], xy1[:,1], '.')

plt.figure('Right Image')
plt.clf()
ax1 = plt.subplot(111)
ax1.imshow(right_img, cmap='gray', interpolation='nearest', vmax=vmax)
ax1.axis('image')
ax1.set_title('Right Image')
ax1.plot(xy2[:,0], xy2[:,1], '.')
#plt.show()    
print xy1.shape, xy2.shape
#modified on 03/21/17 
xy1_filt = xy1[(xy2[:,1]<=1.075*xy1[:,1]) & (xy2[:,1]>=.925*xy1[:,1])]
xy2_filt = xy2[(xy2[:,1]<=1.075*xy1[:,1]) & (xy2[:,1]>=.925*xy1[:,1])]

w1=2048
h1=1536
w2=w1
h2=h1
view3 = sp.zeros((max(h1, h2), w1 + w2), sp.uint8)
view3[:h1, :w1] = left_img  
view3[:h2, w1:] = right_img
view3[:, :] = view3[:, :]  
view3[:, :] = view3[:, :]
for m in xrange(len(xy1_filt)):
    #len(xy1_filt)):
    # draw the keypoints
    color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
    cv2.line(view3, (int(xy1_filt[m,0]), int(xy1_filt[m,1])),(int(xy2_filt[m,0] + w1), int(xy2_filt[m,1])), color)

cv2.namedWindow('filtered',cv2.WINDOW_NORMAL)
cv2.resizeWindow('filtered', 1500,600)
cv2.imshow("filtered", view3)
cv2.waitKey()

# Calculate the 3D points based on the features given
left_points = np.array([xy1_filt[pt] for pt in range(len(xy1_filt))], dtype=float)
right_points = np.array([xy2_filt[pt] for pt in range(len(xy2_filt))], dtype=float)
x1 = cv2.undistortPoints(left_points[None,:,:],cal_data['cameraMatrix1'],
                         cal_data['distCoeffs1'],
                         R=cal_data['R1'], P=cal_data['P1'])[0]
x2 = cv2.undistortPoints(right_points[None,:,:],
                         cal_data['cameraMatrix2'],
                         cal_data['distCoeffs2'],
                         R=cal_data['R2'], P=cal_data['P2'])[0]
                         
points4d = cv2.triangulatePoints(cal_data['P1'], cal_data['P2'], x1.T, x2.T)
points3d = (points4d[:3]/points4d[3, :]).T
print points3d.shape
points3d_filt=points3d[(points3d[:,2]>=0) & (points3d[:,2]<50000)]#,
print points3d_filt.shape
# Show the results
fig = plt.figure('3D Plot of Points')
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points3d_filt[:,0], points3d_filt[:,2], -points3d_filt[:,1], depthshade=False)
#ax.scatter(points3d_filt[:,0], points3d_filt[:,2], -points3d_filt[:,1], c='r',marker='o')
ax.set_xlabel('Horizontal')
ax.set_ylabel('Depth')
ax.set_zlabel('Vertical')
ax.set_title('3D Representation of Points from Stereo Reconstruction')
ax.set_xlim(-1500, 1500)
ax.set_ylim(1000, 4000)
ax.set_zlim(-1500, 1500)
scaling = np.array(
    [getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)


disparity=list()
for m in xrange(0,len(xy1_filt)):
        diff = (xy1_filt[m,0] - xy2_filt[m,0])
        disparity.append(diff)        

'''
disparity_plot = sp.zeros((h1, w1), sp.uint8)
disparity_plot[:h1, :w1] = left_img 
for m in range(len(disparity)):
    color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
    cv2.line(disparity_plot, (xy1_filt[m,0],xy1_filt[m,1]), (xy1_filt[m,0]+disparity[m],xy1_filt[m,1]), color)

cv2.namedWindow('disparity',cv2.WINDOW_NORMAL)
cv2.resizeWindow('disparity', 750,600)
cv2.imshow("disparity", disparity_plot)
cv2.waitKey()'''

plt.show()

#print points3d
#mean_depth = (sum(points3d[:,2]) / len(points3d[:,2]))
#post process and outputs
print '#keypoints in image1: %d, image2: %d' % (len(kp1), len(kp2))
    # visualize the matches
print '#matches:', len(matches)
print 'distance: min: %.3f' % min(dist)
print 'distance: mean: %.3f' % (sum(dist) / len(dist))
print 'distance: max: %.3f' % max(dist)
print 'standard deviation in depth: %.3f' % np.std(dist)
print '#selected matches:', len(sel_matches)
print '#filtered matches:', len(xy1_filt)
print '#filtered triangulated:', len(points3d_filt[:,2])
print 'average depth in mm is', sum(points3d_filt[:,2])/len(points3d_filt[:,2])   
    
'''    # Draw first X matches.
    #images sorted in order of their distance
sel_matches_sort =sorted(sel_matches, key = lambda x:x.distance)
img3 = cv2.drawMatches(left_img,kp1,right_img,kp2,sel_matches_sort[:160],None,flags=2)
cv2.namedWindow('view2',cv2.WINDOW_NORMAL)
cv2.resizeWindow('view2', 1500,600)
cv2.imshow("view2", img3)
cv2.waitKey()'''
   
if print_file:
    #separators=[pos for pos, char in enumerate(LEFT_FILE) if char == '/']
    #print_name='cal_'+LEFT_FILE[separators[-1]+1:len(filename_R)-4]+'.png'
    print_name='cal_'+LEFT_FILE+'.png'      
    #cv2.imwrite(print_name,rectified)#change name to save a given output image
    
if point_cloud:
    print_name='calrev2_'+LEFT_FILE+'.out'         
    np.savetxt(print_name, (points3d))
   
if thicker_line:
    left_img = plt.imread(os.path.join(DATA_DIR1, LEFT_FILE))[:, :, 0]
    #print left_img
    left_img=cv2.cvtColor(left_img,cv2.COLOR_GRAY2RGB)
    #color = [(139, 0, 0), 
          #(0, 100, 0),
          #(0, 0, 139)]
    #disparity_plot = sp.zeros((h1, w1), sp.uint8)
    #disparity_plot[:h1, :w1] = left_img
    if len(left_img.shape) == 3:
        new_shape = (left_img.shape[0], left_img.shape[1], left_img.shape[2])
    elif len(left_img.shape) == 2:
        new_shape = (left_img.shape[0], left_img.shape[1])
    new_img = np.zeros(new_shape, type(left_img.flat[0]))  
    # Place images onto the new image.
    new_img[0:left_img.shape[0],0:left_img.shape[1]] = left_img
    # Draw lines between matches.  Make sure to offset kp coords in second image appropriately.
    r = 5
    thickness = 2
    if color:
        c = color
    for m in matches:
        # Generate random color for RGB/BGR and grayscale images as needed.
        if not color: 
            c = np.random.randint(0,256,3) if len(left_img.shape) == 3 else np.random.randint(0,256)
    for m in range(len(disparity)):
        c = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
        cv2.line(new_img, (xy1_filt[m,0],xy1_filt[m,1]), (xy1_filt[m,0]+disparity[m],xy1_filt[m,1]), c, thickness)
        cv2.circle(new_img, (xy1_filt[m,0],xy1_filt[m,1]), r, c, thickness)
        cv2.circle(new_img, (xy1_filt[m,0]+disparity[m],xy1_filt[m,1]), r, c, thickness)
    
    cv2.namedWindow('disparity',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('disparity', 1000,600)
    cv2.imshow("disparity", new_img)
    cv2.waitKey()
    
target_center = np.mean(points3d_filt, axis=0)
#print tmp
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


