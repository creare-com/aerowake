# -*- coding: utf-8 -*-
"""
Created on Mon Jan 16 10:54:27 2017

@author: fiv
"""

"""
Created on Thu Sep 29 16:12:57 2016
@author: fiv
"""

# matching features of two images
import cv2
import scipy as sp
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

'''for timing purposes, could also use a python function called timeit or similar'''
#import time
#start_time = time.time()
#for i in range(0,999):

"""define image pair"""
#6755 left, 6575 right

filename_R='//Olympus/Projects/6598-Optical-Guide-II/Technical Work/Testing/Data/20170117 - Stereo Cal Images/Right9.tif'
filename_L='//Olympus/Projects/6598-Optical-Guide-II/Technical Work/Testing/Data/20170117 - Stereo Cal Images/Left9.tif'
#filename_R='C:/Users/fiv/Desktop/optical guide/r-6575/6_4_R4.png'
#filename_L='C:/Users/fiv/Desktop/optical guide/l-6755/6_4_L4.png'
#filename_R='C:/Users/fiv/Desktop/optical guide/Optical Guide II/6575/out3.png'
#filename_L='C:/Users/fiv/Desktop/optical guide/Optical Guide II/6755/out3.png'

separators=[pos for pos, char in enumerate(filename_R) if char == '/']
#print filename_R[separators[-1]+1:len(filename_R)]

left_img = cv2.imread(filename_L,0)
right_img = cv2.imread(filename_R,0)
original = np.concatenate((left_img, right_img), axis=1)
cv2.namedWindow('original',cv2.WINDOW_NORMAL)
cv2.resizeWindow('original', 1500,600)
cv2.imshow("original", original)
#cv2.waitKey()


#image_size = left_img.shape
[height,width]= left_img.shape
print left_img.shape
#Calibration matrices
cameraMatrix1=np.array([[1.05612125e+3, 0e+0, 1.01797316e+3],
                [0.0e0, 1.05612125e+3, 7.74008007e+2],
                [0.0e0, 0.0e0, 1.0e0]])
#print(cameraMatrix1)
cameraMatrix2=np.array([[1.0581597e+3, 0e+0, 1.00633871e+3],
                [0.0e0, 1.05815970e+3, 7.32887003e2],
                [0.0e0, 0.0e0, 1.0e0]])
#print(cameraMatrix2)                
distCoeffs1=np.array([-0.32260017,0.10400534,0.00083202,0.00041869,
              -0.01483929,0.0,0.0,0.0])
#print(distCoeffs1)
distCoeffs2=np.array([-.36100603,0.17792664,0.00062531,0.00078929,
              -0.05002994,0.0,0.0,0.0])
#print(distCoeffs2)
#imageSize=[2048,1536]
#width=2048
#height=1536
R=np.array([[9.9997655e-1,-4.07412041e-3,5.50471991e-3],
   [4.07838771e-3,9.99991391e-1,-7.64204731e-4],
    [-5.50155906e-3,7.86637192e-4,9.99984557e-1]])
T=np.array([[-160.32546955],[-0.30323083],[-2.59238926]])

rectify_scale = 0 # 0=full crop, 1=no crop
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (width, height), R, T, alpha = rectify_scale)

left_maps = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (width, height), cv2.CV_16SC2)
right_maps = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (width, height), cv2.CV_16SC2)
 
left_img_remap = cv2.remap(left_img, left_maps[0], left_maps[1], cv2.INTER_LANCZOS4)#, left_img_remap, cv2.BORDER_CONSTANT, 0)
right_img_remap = cv2.remap(right_img, right_maps[0], right_maps[1], cv2.INTER_LANCZOS4)#LINEAR


rectified = np.concatenate((left_img_remap, right_img_remap), axis=1)
cv2.namedWindow('rectified',cv2.WINDOW_NORMAL)
cv2.resizeWindow('rectified', 1500,600)
cv2.imshow("rectified", rectified)
cv2.waitKey()

print_file=0 #if selected matches image desired
point_cloud=0 #if point cloud file desired (format x,y,z), where z is depth
#filename_L='C:/Users/fiv/Documents/python Scripts/depth map/navcam/l/NLB_526667539EDR_F0572798NCAM00251M_.JPG'
#filename_R='C:/Users/fiv/Documents/python Scripts/depth map/navcam/r/NRB_526667539EDR_F0572798NCAM00251M_.JPG'
img1 = left_img_remap#cv2.imread(filename_L,0)
img2 = right_img_remap#cv2.imread(filename_R,0)

detector = cv2.FastFeatureDetector_create()
cv2.ocl.setUseOpenCL(False)
descriptor = cv2.ORB_create()

#http://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html#bruteforcematcher
#suggests Hamming2 is better with ORB, I was previously using Hamming
matcher = cv2.DescriptorMatcher_create("BruteForce-Hamming")
#matcher = cv2.DescriptorMatcher_create("BruteForce")

# detect keypoints
kp1 = detector.detect(img1)
kp2 = detector.detect(img2)

(kp1, d1) = descriptor.compute(img1, kp1)
(kp2, d2) = descriptor.compute(img2, kp2)
# match the keypoints
matches = matcher.match(d1, d2)
# threshold: half the mean
dist = [m.distance for m in matches]    
thres_dist = (sum(dist) / len(dist)) * 0.75
#print((sum(dist) / len(dist)))
# keep only the reasonable matches
sel_matches = [m for m in matches if m.distance < thres_dist]

#end_time=time.time()
#process_time=(end_time-start_time)/1000
#print("--- %s seconds ---" % process_time)

#post process and outputs
print '#keypoints in image1: %d, image2: %d' % (len(kp1), len(kp2))
# visualize the matches
print '#matches:', len(matches)
print 'distance: min: %.3f' % min(dist)
print 'distance: mean: %.3f' % (sum(dist) / len(dist))
print 'distance: max: %.3f' % max(dist)

print '#selected matches:', len(sel_matches)

# visualization of the matches
h1, w1 = img1.shape[:2]
h2, w2 = img2.shape[:2]
view = sp.zeros((max(h1, h2), w1 + w2), sp.uint8)
view[:h1, :w1] = img1  
view[:h2, w1:] = img2
view[:, :] = view[:, :]  
view[:, :] = view[:, :]

for m in sel_matches:
    # draw the keypoints
    # print m.queryIdx, m.trainIdx, m.distance
    color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
    cv2.line(view, (int(kp1[m.queryIdx].pt[0]), int(kp1[m.queryIdx].pt[1])) , (int(kp2[m.trainIdx].pt[0] + w1), int(kp2[m.trainIdx].pt[1])), color)

cv2.namedWindow('view1',cv2.WINDOW_NORMAL)
cv2.resizeWindow('view1', 1500,600)
cv2.imshow("view1", view)
cv2.waitKey()

'''# Draw first X matches.
#images sorted in order of their distance
sel_matches_sort =sorted(sel_matches, key = lambda x:x.distance)
img3 = cv2.drawMatches(img1,kp1,img2,kp2,sel_matches_sort[:160],None,flags=2)

cv2.namedWindow('view2',cv2.WINDOW_NORMAL)
cv2.resizeWindow('view2', 1500,600)
cv2.imshow("view2", img3)
cv2.waitKey()'''

'''y coordinate masking'''
xy1=list()
xy2=list()
for m in sel_matches:
    xy1_temp = kp1[m.queryIdx].pt
    xy2_temp = kp2[m.trainIdx].pt
    xy1.append(xy1_temp)
    xy2.append(xy2_temp)

#xy1 = [kp1[mat.queryIdx].pt for mat in matches] 
#xy2 = [kp2[mat.trainIdx].pt for mat in matches]    
#tuple to np array for masking purposes
xy1=np.array(xy1)
xy1.flatten
xy2=np.array(xy2)
xy2.flatten

xy_sel1 = xy1[(xy2[:,1]<=1.05*xy1[:,1]) & (xy2[:,1]>=.95*xy1[:,1])]
xy_sel2 = xy2[(xy2[:,1]<=1.05*xy1[:,1]) & (xy2[:,1]>=.95*xy1[:,1])]
print '#filtered matches:', len(xy_sel1)

'''view3 = sp.zeros((max(h1, h2), w1 + w2), sp.uint8)
view3[:h1, :w1] = img1  
view3[:h2, w1:] = img2
view3[:, :] = view3[:, :]  
view3[:, :] = view3[:, :]
for m in xrange(100,200):
#len(xy_sel1)):
    # draw the keypoints
    # print m.queryIdx, m.trainIdx, m.distance
    color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
    cv2.line(view3, (int(xy_sel1[m,0]), int(xy_sel1[m,1])),(int(xy_sel2[m,0] + w1), int(xy_sel2[m,1])), color)

cv2.namedWindow('view3',cv2.WINDOW_NORMAL)
cv2.resizeWindow('view3', 1500,600)
cv2.imshow("view3", view3)
cv2.waitKey()'''

'''depth calculation'''
focal_length=1057.978466
baseline=160 #distance between cameras in mm
triangulation_constant=focal_length*baseline

'''only if scattered figure desired
plt.ion()
fig=plt.figure()'''
'''depth will be printed in m'''
#xy=list()
depth=list()
for m in xrange(0,len(xy_sel1)):
        #left_pt = kp1[m.queryIdx].pt
        #right_pt = kp2[m.trainIdx].pt
        #dispartity = abs(left_pt[0] - right_pt[0])
        dispartity = abs(xy_sel1[m,0] - xy_sel2[m,0])
        z = triangulation_constant / dispartity
        depth.append(z)        
        '''        
        plt.scatter(i,z)
        plt.show()
        plt.pause(0.0001)'''
xyz = np.column_stack((xy_sel1,depth))  
print xyz  
plt.plot(depth)
plt.show()
mean_depth = (sum(depth) / len(depth))
print 'average depth in mm is', mean_depth

if print_file:
    print_name='cal_'+filename_R[separators[-1]+1:len(filename_R)-4]+'.png'      
    cv2.imwrite(print_name,rectified)
    
if point_cloud:
    print_name='calrev2_'+filename_R[separators[-1]+1:len(filename_R)-4]+'.out'         
    np.savetxt(print_name, (xyz))
#, ('b', '^', -30, -5)]:
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xyz[:,0], xyz[:,2], -xyz[:,1], c='r',marker='o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()

print np.ptp(xyz,axis=1)
#print range(xyz[:,0]),range(xyz[:,1])

