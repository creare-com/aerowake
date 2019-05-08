# -*- coding: utf-8 -*-
"""
Created on Mon Jan 16 10:54:27 2017

@author: fiv
"""

"""
Created on Thu Sep 29 16:12:57 2016
@author: fiv
Latest modifications: (12/4/17), applying clockwise rotation matrix, angle 19.63 deg
in the x-depth plane. 

In this revision we will clean up the code, features are found in undistorted space, matched.  
Matched features are then triangulated.
This is a cleaned up version from "ptc_cloud_via_kpts_binning_V4_addedfilters_no_plots_timer.py".  
Modifications include: image is broken up in y space.  keypoints are then binned based on a user 
defined # of division.  Additional work needed to increase the number of matches as 
number of divisions is increased can be included by modifying the matching space on the train image
(left image).  Following work will attempt this.
User needs to define: DATA_DIR, image file number
Currently using example files in "Creare_Tour" series
In this version we are reprojecting all points towards a commong floor plane, 
still unsure if this should be the correct way to do it, or if the points should
have been automatically coorrected by rectification/triangulation algorithsms.
"""

import cv2
import os
from matplotlib import pyplot as plt
import numpy as np
import plot_functions
#import find_homography
from mpl_toolkits.mplot3d import Axes3D
import itertools
import time

"""define image pair"""
#6755 left, 6575 right
#CAL_FILE = 'stereo_cal_2017-07-03.npz'
#CAL_FILE = 'stereo_ex2_cal.npz'
DATA_DIR = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                        'Technical Work', 'Testing', 'Data',
                        '20170614 - Stereo loop to patio','startup_rec_data')

'''DATA_DIR = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                        'Technical Work', 'Testing', 'Data',
                        'rec_2017-07-28_173316_jdw_walking','copies')'''
print DATA_DIR
#number of images in a file, needed in case for loop is defined based on range
file_len=int(0.5*(len(os.walk(DATA_DIR).next()[2])-1))

point_cloud=0 # if 1 point cloud data is printed to text file
drawlines=True

#feature detection functions defined
detector = cv2.FastFeatureDetector_create()
cv2.ocl.setUseOpenCL(False)
descriptor = cv2.ORB_create()
# create BFMatcher object
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
#crossCheck not necessary false by default and only works with bfmatcher not with knn.Match
CAL_FILE = 'stereo_cal_2017-07-03.npz'
# Load the required data
cal_data = np.load(CAL_FILE)
[height,width]=[1536,2048]
#matrices needed for rectification performed only once
left_maps = cv2.initUndistortRectifyMap(cal_data['cameraMatrix1'], cal_data['distCoeffs1'],
                                        cal_data['R1'], cal_data['P1'], (width, height), cv2.CV_16SC2)
right_maps = cv2.initUndistortRectifyMap(cal_data['cameraMatrix2'], cal_data['distCoeffs2'],
                                        cal_data['R2'], cal_data['P2'], (width, height), cv2.CV_16SC2)
                                        
ii=25###defining image to analyze
#LEFT_FILE='Left ('+str(ii)+').tif'
#RIGHT_FILE='Right ('+str(ii)+').tif'
LEFT_FILE='leftVisible_2016-05-06_054454_367.tif'
RIGHT_FILE='rightVisible_2016-05-06_054454_367.tif'


start_time = time.time()#only needed if interested
#importing image
left_img_orig = plt.imread(os.path.join(DATA_DIR, LEFT_FILE))
right_img_orig = plt.imread(os.path.join(DATA_DIR, RIGHT_FILE))
#undistorting/rectifying image
cv2.imwrite('original left image.png', left_img_orig)
imgL = cv2.remap(left_img_orig, left_maps[0],
                 left_maps[1], cv2.INTER_CUBIC)# cv2.INTER_LANCZOS4)
imgR = cv2.remap(right_img_orig, right_maps[0], right_maps[1],
                 cv2.INTER_CUBIC)# cv2.INTER_LANCZOS4)
cv2.imwrite('undistorted left image.png', imgL)
#passing filter to remove noise
left_img = cv2.GaussianBlur(imgL, (3,3),3)
right_img = cv2.GaussianBlur(imgR, (3,3),3)

# detect keypoints
kp1 = detector.detect(left_img)
kp2 = detector.detect(right_img)
kp1=sorted(kp1, key=lambda kp1: kp1.pt[1])#sorts kp1 based on y coordinate
kp2=sorted(kp2, key=lambda kp2: kp2.pt[1])#sorts kp1 based on y coordinate

# Initialize lists
coord_kp1 = []
coord_kp2 = []
# For each kpt...
for mat in range(0,len(kp1)-1):
    # Get the matching keypoints for each of the images
    (x1,y1) = kp1[mat].pt
    # Append to each list
    coord_kp1.append((x1, y1))
for mat in range(0,len(kp2)-1):
    # Get the matching keypoints for each of the image
    (x2,y2) = kp2[mat].pt  
    coord_kp2.append((x2, y2))    

#we find the limits (binning y coordinate limits) defining 
#the binning edges; based on max y =1536, we bin based on 1536/divisions
coord_kp1=np.array(coord_kp1)
coord_kp2=np.array(coord_kp2)
kp_bin1=[]
kp_bin1.append(0)
kp_bin2=[]
kp_bin2.append(0)
divisions=6 #number of axial (y) bins to break the image into
bin_pixel_length=(height + divisions // 2) // divisions
for i in range(1,divisions):
    kp_bin1.append(np.where(coord_kp1[:, -1] > i*bin_pixel_length)[0][0])
    kp_bin2.append(np.where(coord_kp2[:, -1] > i*bin_pixel_length)[0][0])
kp_bin1.append(len(kp1))
kp_bin2.append(len(kp2))
#kp_bin1 contains indices for kp1, kp_bin2 contains indices for kp2

xyz=np.array([]).reshape(0,3)
xy1=np.array([]).reshape(0,2)
xy2=np.array([]).reshape(0,2)
for i in range(0,divisions):
    #declaring
    kpp1=[];     kpp2=[]; d1=[]; d2=[]; 
    xy1_temp=np.array([]).reshape(0,2);xy2_temp=np.array([]).reshape(0,2);
    #descriptor computation
    (kpp1, d1) = descriptor.compute(left_img, kp1[kp_bin1[i]:kp_bin1[i+1]-1])
    (kpp2, d2) = descriptor.compute(right_img, kp2[kp_bin2[i]:kp_bin2[i+1]-1])
    sel_matches_temp = [] 
    #knn matche
    matches = matcher.knnMatch(d1, d2,k=2)
    #1st filtering, ration test    
    for m,n in matches:
         if m.distance < 0.75*n.distance:
             # Removed the brackets around m 
             sel_matches_temp.append(m)
    
    for w in sel_matches_temp:
        xy1_temp=np.vstack([xy1_temp,kpp1[w.queryIdx].pt])
        xy2_temp=np.vstack([xy2_temp,kpp2[w.trainIdx].pt])
        #xy1_temp.append(kpp1[w.queryIdx].pt)
        #xy2_temp.append(kpp2[w.trainIdx].pt)
    #xy1_temp=np.asarray(xy1_temp).astype(np.float32)
    #xy2_temp=np.asarray(xy2_temp).astype(np.float32) 

    xy1=np.vstack([xy1,xy1_temp])
    xy2=np.vstack([xy2,xy2_temp])
 
##Removing matches with negative disparity or very low disparity
#which would lead to far away points,     
disparity=list()
for m in xrange(0,len(xy1)):
    diff = (xy1[m,0] - xy2[m,0])
    disparity.append(diff) 

left_points = [];right_points = []
for i in xrange(len(disparity)):
    if disparity[i] > 3:
            # Removed the brackets around m 
        left_points.append(xy1[i,:])
        right_points.append(xy2[i,:])
xy11 = np.array([left_points[pt] for pt in range(len(left_points))], dtype=float)
xy2 = np.array([right_points[pt] for pt in range(len(right_points))], dtype=float) 
    
#Masking x,y space, might consider removing, modifying, or testing for robustness, 
#might consider masking some percentage of the image borders...
xy1 = xy11[(xy11[:,0]>200)&(xy11[:,0]<1900)]
xy2 = xy2[(xy11[:,0]>200)&(xy11[:,0]<1900)]
xy2 = xy2[(xy1[:,1]>150)&(xy1[:,1]<1450)]  
xy1= xy1[(xy1[:,1]>150)&(xy1[:,1]<1450)]

#triangulation
points4d = cv2.triangulatePoints(cal_data['P1'], cal_data['P2'], xy1.T, xy2.T)
points3d = (points4d[:3]/points4d[3, :]).T
xyz=np.vstack([xyz,points3d])

angle=19.63*np.pi/180
xyz_rotx=np.array([])
xyz_rotz=np.array([])
xyz_rotx=(xyz[:,0]*np.cos(angle)+xyz[:,2]*np.sin(angle))
xyz_rotz=(xyz[:,0]*-np.sin(angle)+xyz[:,2]*np.cos(angle))
xyz[:,0]=xyz_rotx
xyz[:,2]=xyz_rotz
#xyz[:,1]=(-xyz[:,1])-((xyz[:,2]-5300)*np.tan(0.1444))

#manually calculated angle of 13.4214 degrees planar offset, (corresponding to 0.23425 radians)
#manually performing this correction for now
#print xyz.shape
end_time=time.time() 
process_time=(end_time-start_time)  
print("--- %s seconds ---" % process_time)
print xyz.shape
plot_functions.detectedkpoints_2pair(left_img,xy1,xy2)
plot_functions.pointcloud(xyz)
if point_cloud:
    print_name=LEFT_FILE+'.out'         
    np.savetxt(print_name, (xyz)) 
    print 'text file written: %s ' %print_name
    
    xyz=np.column_stack((np.column_stack((xyz[:,0],xyz[:,2])),xyz[:,1]))
    print xyz.shape


#plt.plot(xyz[:,2]/1000, xyz[:,1]/1000,".")

vmax = 80
fig=plt.figure()
ax1 = fig.add_subplot(111,projection='3d')#aspect='equal', )
ax1.scatter(xyz[:,0]/1000, xyz[:,2]/1000, xyz[:,1]/1000, depthshade=False)
#ax1.view_init(0, -90) #front view of x,z plane
#ax1.axis('equal')
#ax1.set_aspect('equal',adjustable='box')
ax1.set_xlim(-12, 12)
#ax1.set_xlim(-17.5, 17.5)
ax1.set_ylim(5, 35)
ax1.set_zlim(0, 11)
ax1.view_init(40, -65)

#ax1.set_aspect('equal', 'datalim')
#ax1.view_init(18, -74)
plt.show()

if drawlines:
    #col = np.random.randint(0,256,3)
    #r,c = imgL.shape
    imgL = cv2.cvtColor(imgL,cv2.COLOR_GRAY2BGR)
    imgR = cv2.cvtColor(imgR,cv2.COLOR_GRAY2BGR)
    new_shape = (max(imgL.shape[0], imgR.shape[0]),
                 imgL.shape[1]+imgR.shape[1], imgL.shape[2])
                 
    # Place images onto the new image.
    new_img = np.zeros(new_shape, type(imgL.flat[0]))  
    new_img[0:imgL.shape[0],0:imgL.shape[1]] = imgL
    new_img[0:imgR.shape[0],imgL.shape[1]:imgL.shape[1]+imgR.shape[1]] = imgR
    thickness = 2
    rad=10
    if len(xy1)%2==0:
            up_to=len(xy1)
    else:
            up_to=len(xy1)-1
                
    for pt in range(0,up_to/12):
    #for pt in range(0,((len(xy1)-1)/2)):
        color = np.random.randint(0,256,3)
        endL=tuple(np.round(xy1[pt*12,:]).astype(int))
        endR=tuple((np.round(xy2[pt*12,0]+imgL.shape[1]).astype(int),
                 np.round(xy2[pt*12,1]).astype(int)))        
        cv2.line(new_img,endL,endR,color, thickness)
        cv2.circle(new_img, endL, rad, color, thickness)
        cv2.circle(new_img, endR, rad, color, thickness)
    
    cv2.namedWindow('matched_lines',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('matched_lines', 500,300)
    cv2.imshow("matched_lines", new_img)
    cv2.waitKey(0)
    cv2.imwrite('L_119_matched-lines.tiff',new_img)
    print (len(xy1)-1/2)-1
    print (len(xy1))    
       
'''        
color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv2.line(img1, (x0,y0), (x1,y1), color,1)
        img1 = cv2.circle(img1,tuple(pt1),5,color,-1)
        img2 = cv2.circle(img2,tuple(pt2),5,color,-1)
        return img1,img2
        
        endL = tuple(np.round(xy1[0,0]).astype(int))
        endR = tuple(np.round(xy2[0,0]).astype(int) + np.array([imgL.shape[1], 0]))
        
        endL = xy1[0,0]
        endR = xy2[0,0] + np.array([imgL.shape[1], 0])
        cv2.line(new_img, endL, endR, color, thickness)
        cv2.circle(new_img, endL, rad, color, thickness)
        cv2.circle(new_img, endR, rad, color, thickness)        
        
        print tuple((np.round(xy2[0,0]+imgL.shape[1]).astype(int),
                 np.round(xy2[0,0]).astype(int))
'''
#print np.column_stack((xyz[:,0], xyz[:,0], xy2z[:,1]))
#xyz=[xyz[:,0]/1000, xyz[:,2]/1000, xyz[:,1]/1000]
'''
original = np.concatenate((imgL, imgR), axis=1)
cv2.namedWindow('original',cv2.WINDOW_NORMAL)
cv2.resizeWindow('original', 1500,600)
cv2.imshow("original", original)
cv2.waitKey()'''