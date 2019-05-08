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
from matplotlib import pyplot as plt
import numpy as np
import plot_functions
from mpl_toolkits.mplot3d import Axes3D
import itertools
import time
"""define image pair"""
#6755 left, 6575 right
CAL_FILE = 'stereo_ex2_cal.npz'

DATA_DIR = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                        'Technical Work', 'Testing', 'Data',
                        '20140405 - Stereo tour of Creare','images_copy')
#print DATA_DIR
file_len=int(0.5*(len(os.walk(DATA_DIR).next()[2])-1))

#feature detection functions defined
detector = cv2.FastFeatureDetector_create()
cv2.ocl.setUseOpenCL(False)
descriptor = cv2.ORB_create()
# create BFMatcher object
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
#crossCheck not necessary false by default and only works with matcher not with knn.Match

# Load the required data
cal_data = np.load(CAL_FILE)
[height,width]=[1536,2048]
rectify_scale = 0 # 0=full crop, 1=no crop

left_maps = cv2.initUndistortRectifyMap(cal_data['cameraMatrix1'], cal_data['distCoeffs1'],
                                        cal_data['R1'], cal_data['P1'], (width, height), cv2.CV_16SC2)
right_maps = cv2.initUndistortRectifyMap(cal_data['cameraMatrix2'], cal_data['distCoeffs2'],
                                        cal_data['R2'], cal_data['P2'], (width, height), cv2.CV_16SC2)
 
#print os.path.join(DATA_DIR, LEFT_FILE)

#fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#out = cv2.VideoWriter('outputFast-orb_point-cloud-knnmatch.avi',fourcc, 5.0,(960,440))
ii=243###defining image to analyze
#for ii in range(273,362):# in range(225,file_len-1):
LEFT_FILE='L '+'('+str(ii)+').tif'
RIGHT_FILE='R '+'('+str(ii)+').tif'
start_time = time.time()
left_img_orig = plt.imread(os.path.join(DATA_DIR, LEFT_FILE))#[:, :, 0]
right_img_orig = plt.imread(os.path.join(DATA_DIR, RIGHT_FILE))#[:, :, 0]
left_img = cv2.remap(left_img_orig, left_maps[0], left_maps[1], cv2.INTER_LINEAR)# cv2.INTER_LANCZOS4)#, left_img_remap, cv2.BORDER_CONSTANT, 0)
right_img = cv2.remap(right_img_orig, right_maps[0], right_maps[1], cv2.INTER_LINEAR)# cv2.INTER_LANCZOS4)#LINEAR
#plot_functions.plot_original(left_img_orig,right_img_orig)
#plot_functions.plot_original(left_img,right_img)
#subplot of original matches
#plot_functions.plot_original(left_img,right_img)

# detect keypoints
kp1 = detector.detect(left_img)
#print len(kp2)
kp2 = detector.detect(right_img)
(kp1, d1) = descriptor.compute(left_img, kp1)
(kp2, d2) = descriptor.compute(right_img, kp2)
    
    # match the keypoints
matches = matcher.knnMatch(d1, d2,k=2)

    # Apply ratio test
    # keep only the reasonable matches, the lower the value of dist the better
sel_matches = []
for m,n in matches:
    if m.distance < 0.75*n.distance:
        # Removed the brackets around m 
        sel_matches.append(m)
       
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

    #modified on 03/21/17 
xy1_filt = xy1[(xy2[:,1]<=1.075*xy1[:,1]) & (xy2[:,1]>=.925*xy1[:,1])]
xy2_filt = xy2[(xy2[:,1]<=1.075*xy1[:,1]) & (xy2[:,1]>=.925*xy1[:,1])]

    # Calculate the 3D points based on the features given
left_points1 = np.array([xy1_filt[pt] for pt in range(len(xy1_filt))], dtype=float)
right_points1 = np.array([xy2_filt[pt] for pt in range(len(xy2_filt))], dtype=float)
disparity=list()
for m in xrange(0,len(left_points1)):
    diff = (left_points1[m,0] - right_points1[m,0])
    disparity.append(diff) 

left_points = [];right_points = []
for i in xrange(len(disparity)):
    if disparity[i] > 3:
            # Removed the brackets around m 
        left_points.append(left_points1[i,:])
        right_points.append(right_points1[i,:])
left_points = np.array([left_points[pt] for pt in range(len(left_points))], dtype=float)
right_points = np.array([right_points[pt] for pt in range(len(right_points))], dtype=float)    
     
x11 = cv2.undistortPoints(left_points[None,:,:],cal_data['cameraMatrix1'],
                         cal_data['distCoeffs1'],
                         R=cal_data['R1'], P=cal_data['P1'])[0]
x2 = cv2.undistortPoints(right_points[None,:,:],
                         cal_data['cameraMatrix2'],
                         cal_data['distCoeffs2'],
                         R=cal_data['R2'], P=cal_data['P2'])[0]
    
#Masking x,y space, might consider removing, modifying, or testing for robustness, 
#might consider masking some percentage of the image borders...
x1 = x11[(x11[:,0]>250)&(x11[:,0]<2000)]
x2 = x2[(x11[:,0]>250)&(x11[:,0]<2000)]
x2 = x2[(x1[:,1]>400)&(x1[:,1]<1500)]  
x1= x1[(x1[:,1]>400)&(x1[:,1]<1500)]
                       
points4d = cv2.triangulatePoints(cal_data['P1'], cal_data['P2'], x1.T, x2.T)
points3d = (points4d[:3]/points4d[3, :]).T
points3d_filt=points3d#[(points3d[:,2]>=0) & (points3d[:,2]<50000)]#
end_time=time.time()
process_time=(end_time-start_time)   
#data=plot_functions.video_subplot(left_img,x1,x2,points3d_filt,ii,1) 
vmax = 80
fig=plt.figure(figsize=(12, 5.5))
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
ax1.set_title('3D Representation of Points from Stereo Reconstruction \n image %s'%str(ii+1))
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
# Show the results

print "----------------------------------------------"     
#post process and outputs
print("--- %s seconds ---" % process_time)
print '#keypoints in image1: %d, image2: %d' % (len(kp1), len(kp2))
print '#matches:', len(matches)
print '#selected matches:', len(sel_matches)
print '#filtered matches:', len(xy1_filt)
print '#filtered triangulated:', len(points3d_filt[:,2])
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

#out.write(data)
print ii
print 'video creation finished at image %d ' %ii
#out.release()
#cv2.destroyAllWindows()  
'''   
#plot_functions.video_subplot(left_img,x1,x2,points3d_filt,259,1)
vmax = 80
fig=plt.figure(figsize=(12, 5.5))
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
ax1.set_title('3D Representation of Points from Stereo Reconstruction \n image %s'%str(ii+1))
ax1.set_xlim(-1500, 1500)
ax1.set_ylim(1000, 4000)
ax1.set_zlim(-1500, 1500)
ax1.view_init(12.5, -60)
fig.tight_layout()
fig.canvas.draw()
plt.show()
    #plt.show()
  
    
 
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
print " - {0:d} features considered".format(len(points3d_filt))
print " - target is estimated to be {0:0.2f}m from the rig".format(stand_off/1000)
print " - RMS deviation from a plane = {0:0.1f}mm".format(rms_error)
'''

  

'''  
    #2 concatenated plots with subplot
    fig=plt.figure(figsize=(12, 5.5))
    plt.clf()
    ax1 = plt.subplot(121)
    vmax = 80    
    ax1.imshow(left_img, cmap='gray', interpolation='nearest', vmax=vmax)
    ax1.axis('image')
    ax1.set_title('Left Image')
    ax1 = fig.add_subplot(122)
    ax1.imshow(left_img, cmap='gray', interpolation='nearest', vmax=vmax)
    ax1.axis('image')
    ax1.set_title('Undistorted Points Plotted on Left Image')
    ax1.plot(x1[:,0], x1[:,1], '.',x2[:,0], x2[:,1], '.')
    leg = plt.legend(['Left', 'Right'],frameon=False, loc = 'best')
    for text in leg.get_texts():
        plt.setp(text, color = 'w')


'''

'''
if point_cloud:
print_name='calrev2_'+LEFT_FILE+'.out'         
np.savetxt(print_name, (points3d))
ax1 = fig.add_subplot(122, projection='3d')
ax1.scatter(points3d_filt[:,0], points3d_filt[:,2], -points3d_filt[:,1], depthshade=False)
ax1.set_xlabel('Horizontal')
ax1.set_ylabel('Depth')
ax1.set_zlabel('Vertical')
ax1.set_title('3D Representation of Points from Stereo Reconstruction \n image %s'%str(ii+1))
ax1.set_xlim(-1500, 1500)
ax1.set_ylim(1000, 4000)
ax1.set_zlim(-1500, 1500)
ax1.view_init(12.5, -60)
'''