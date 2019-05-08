# -*- coding: utf-8 -*-
"""
Created on Mon Jan 16 10:54:27 2017

@author: fiv
"""

"""
Created on Thu Sep 29 16:12:57 2016
@author: fiv
In this revision we will try to add Bryn's triangulation steps,
we will plot Bryn's manually choosen features and try to compare how different they are
from the ones extracted from Brute force matcher
This will be done on a distorted space
"""

# matching features of two images
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
DATA_DIR = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                        'Technical Work', 'Testing', 'Data',
                        '20170117 - Stereo Cal Images')
LEFT_FILE = 'Left9.tif'
RIGHT_FILE = 'Right9.tif'

# Load the required data
cal_data = np.load(CAL_FILE)
left_img = plt.imread(os.path.join(DATA_DIR, LEFT_FILE))[:, :, 0]
right_img = plt.imread(os.path.join(DATA_DIR, RIGHT_FILE))[:, :, 0]

#filename_R='//Olympus/Projects/6598-Optical-Guide-II/Technical Work/Testing/Data/20170117 - Stereo Cal Images/Right9.tif'
#filename_L='//Olympus/Projects/6598-Optical-Guide-II/Technical Work/Testing/Data/20170117 - Stereo Cal Images/Left9.tif'
#separators=[pos for pos, char in enumerate(filename_R) if char == '/']


#left_img = cv2.imread(filename_L,0)
#right_img = cv2.imread(filename_R,0)
original = np.concatenate((left_img, right_img), axis=1)
cv2.namedWindow('original',cv2.WINDOW_NORMAL)
cv2.resizeWindow('original', 1500,600)
cv2.imshow("original", original)
cv2.waitKey()

#Bryn's manually defined features
features = [[(923, 761), (857, 712)],   # Left first, Right second
            [(587, 418), (538, 371)],
            [(1292, 398), (1235, 347)],
            [(1336, 1139), (1273, 1094)],
            [(560, 1146), (505, 1090)],
            [(1314, 742), (1252, 693)],
            [(558, 768), (504, 717)],
            [(919, 398), (861, 350)],
            [(934, 1142), (867, 1092)]]
            
vmax = 80
plt.figure('Raw Data')
plt.clf()
ax1 = plt.subplot(121)
ax1.imshow(left_img, cmap='gray', interpolation='nearest', vmax=vmax)
ax1.axis('image')
ax1.set_title('Left Image')
ax2 = plt.subplot(122)
ax2.imshow(right_img, cmap='gray', interpolation='nearest', vmax=vmax)
ax2.axis('image')
ax2.set_title('Right Image')
for pts in features:
    ax1.plot(pts[0][0], pts[0][1], 'o')
    ax2.plot(pts[1][0], pts[1][1], 'o')          
#plt.show()            
#image_size = left_img.shape
'''[height,width]= left_img.shape
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

R=np.array([[9.9997655e-1,-4.07412041e-3,5.50471991e-3],
   [4.07838771e-3,9.99991391e-1,-7.64204731e-4],
    [-5.50155906e-3,7.86637192e-4,9.99984557e-1]])
T=np.array([[-160.32546955],[-0.30323083],[-2.59238926]])

rectify_scale = 0 # 0=full crop, 1=no crop
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(cameraMatrix1, 
                                                  distCoeffs1, cameraMatrix2, 
                                                  distCoeffs2, (width, height), 
                                                  R, T, alpha = rectify_scale)'''

'''
left_maps = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (width, height), cv2.CV_16SC2)
right_maps = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (width, height), cv2.CV_16SC2)
left_img_remap = cv2.remap(left_img, left_maps[0], left_maps[1], cv2.INTER_LANCZOS4)#, left_img_remap, cv2.BORDER_CONSTANT, 0)
right_img_remap = cv2.remap(right_img, right_maps[0], right_maps[1], cv2.INTER_LANCZOS4)#LINEAR
rectified = np.concatenate((left_img_remap, right_img_remap), axis=1)
cv2.namedWindow('rectified',cv2.WINDOW_NORMAL)
cv2.resizeWindow('rectified', 1500,600)
cv2.imshow("rectified", rectified)
cv2.waitKey()'''

print_file=0 #if selected matches image desired
point_cloud=0 #if point cloud file desired (format x,y,z), where z is depth
img1 = left_img#cv2.imread(filename_L,0)
img2 = right_img#cv2.imread(filename_R,0)

#feature detection functions defined
detector = cv2.FastFeatureDetector_create()
cv2.ocl.setUseOpenCL(False)
descriptor = cv2.ORB_create()
matcher = cv2.DescriptorMatcher_create("BruteForce-Hamming")

# detect keypoints
kp1 = detector.detect(img1)
kp2 = detector.detect(img2)

(kp1, d1) = descriptor.compute(img1, kp1)
(kp2, d2) = descriptor.compute(img2, kp2)
# match the keypoints
matches = matcher.match(d1, d2)
# threshold: half the mean
dist = [m.distance for m in matches]  

print min(dist)  
print np.std(dist)
thres_dist = (sum(dist) / len(dist)) * 0.5
# keep only the reasonable matches
sel_matches = [m for m in matches if m.distance < thres_dist]

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

xy_sel1 =xy1# xy1[(xy2[:,1]<=1*xy1[:,1]) & (xy2[:,1]>=1*xy1[:,1])]
xy_sel2 =xy2# xy2[(xy2[:,1]<=1*xy1[:,1]) & (xy2[:,1]>=1*xy1[:,1])]
print xy_sel1.shape
#print xy_sel1[None,:,:]
#t=[219,220,224]
t=[176,200,210,219,220,224,300,312,345,380,386,530]#,280,313,,415,460,525, range(381,)
#t=[176,200,210,219,220,224,280,300,312,313,345,380,386,415,460,525,530]
plt.figure('Raw Data')
plt.clf()
ax1 = plt.subplot(121)
ax1.imshow(left_img, cmap='gray', interpolation='nearest', vmax=vmax)
ax1.axis('image')
ax1.set_title('Left Image')
ax2 = plt.subplot(122)
ax2.imshow(right_img, cmap='gray', interpolation='nearest', vmax=vmax)
ax2.axis('image')
ax2.set_title('Right Image')
for pts in t:
    ax1.plot(xy_sel1[pts,0], xy_sel1[pts,1], 'o')
    ax2.plot(xy_sel2[pts,0], xy_sel2[pts,1], 'o')         
    
#vmax = 80
fig=plt.figure('Left Image')
plt.clf()
ax1 = plt.subplot(111)
ax1.imshow(left_img, cmap='gray', interpolation='nearest', vmax=vmax)
ax1.axis('image')
ax1.set_title('Left Image')
ax1.plot(xy_sel1[:,0], xy_sel1[:,1], '.')

plt.figure('Right Image')
plt.clf()
ax1 = plt.subplot(111)
ax1.imshow(right_img, cmap='gray', interpolation='nearest', vmax=vmax)
ax1.axis('image')
ax1.set_title('Right Image')
ax1.plot(xy_sel2[:,0], xy_sel2[:,1], '.')
#plt.show()    

# Calculate the 3D points based on the features given
print x2
left_points = np.array([xy_sel1[pt] for pt in t], dtype=float)
right_points = np.array([xy_sel2[pt] for pt in t], dtype=float)
x1 = cv2.undistortPoints(left_points[None,:,:],cal_data['cameraMatrix1'],
                         cal_data['distCoeffs1'],
                         R=cal_data['R1'], P=cal_data['P1'])[0]
x2 = cv2.undistortPoints(right_points[None,:,:],
                         cal_data['cameraMatrix2'],
                         cal_data['distCoeffs2'],
                         R=cal_data['R2'], P=cal_data['P2'])[0]
                         
points4d = cv2.triangulatePoints(cal_data['P1'], cal_data['P2'], x1.T, x2.T)
points3d = (points4d[:3]/points4d[3, :]).T
print points3d

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
'''plane_normal, target_center, _ = best_plane(points3d)
stand_off = np.linalg.norm(target_center)
deviation = np.dot(plane_normal, (points3d - target_center).T)
rms_error = np.sqrt(np.mean(deviation**2))
print "Results of Stereo on Manually Matched Features"
print "----------------------------------------------"
print " - {0:d} features considered".format(len(points3d))
print " - target is estimated to be {0:0.2f}m from the rig".format(
    stand_off/1000)
print " - RMS deviation from a plane = {0:0.1f}mm".format(rms_error)'''

fig = plt.figure('Another 3D plot format')
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points3d[:,0], points3d[:,2], -points3d[:,1], c='r',marker='o')
ax.set_xlabel('Horizontal')
ax.set_ylabel('Depth')
ax.set_zlabel('Vertical')
ax.set_xlim(-1000, 1000)
ax.set_ylim(0, 5000)
ax.set_zlim(-1000, 1000)
#ax.view_init(-85, -90)	

plt.show()

#print points3d

'''
# visualization of the matches
h1, w1 = img1.shape[:2]
h2, w2 = img2.shape[:2]
view_line = sp.zeros((max(h1, h2), w1 + w2), sp.uint8)
view_line[:h1, :w1] = img1  
view_line[:h2, w1:] = img2
view_line[:, :] = view_line[:, :]  
view_line[:, :] = view_line[:, :]

for t in sel_matches:
    # draw the keypoints
    # print m.queryIdx, m.trainIdx, m.distance
    color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
    cv2.line(view_line, (int(kp1[m.queryIdx].pt[0]), int(kp1[m.queryIdx].pt[1])) , (int(kp2[m.trainIdx].pt[0] + w1), int(kp2[m.trainIdx].pt[1])), color)

cv2.namedWindow('view_line',cv2.WINDOW_NORMAL)
cv2.resizeWindow('view_line', 1500,600)
cv2.imshow("view_line", view_line)
cv2.waitKey()

'''
# Draw first X matches.
#images sorted in order of their distance
#sel_matches_sort =sorted(sel_matches, key = lambda x:x.distance)
print sel_matches[t[0]]
print sel_matches[pt]
sel_matches_sort=list()
for pt in t:
    temp=sel_matches[pt]
    sel_matches_sort.append(temp)
print sel_matches_sort
draw_lines = cv2.drawMatches(img1,kp1,img2,kp2,sel_matches_sort[:],None,flags=2)

cv2.namedWindow('draw_lines',cv2.WINDOW_NORMAL)
cv2.resizeWindow('draw_lines', 1500,600)
cv2.imshow("draw_lines", draw_lines)
cv2.waitKey()  
cv2.imwrite('draw_lines_selected_pts.png',draw_lines)

target_center = np.mean(points3d, axis=0)
#print tmp
    # Calculate the plane normal
tmp = points3d - target_center
[D, V] = np.linalg.eig(np.dot(tmp.T, tmp))
V = V  # match MATLAB.
I = np.argsort(np.abs(D))
plane_normal = V[:, I[0]]
n2 = V[:, I[-1]]

stand_off = np.linalg.norm(target_center)
deviation = np.dot(plane_normal, (points3d - target_center).T)
rms_error = np.sqrt(np.mean(deviation**2))
print "Results of Stereo on Manually Matched Features"
print "----------------------------------------------"
print " - {0:d} features considered".format(len(points3d))
print " - target is estimated to be {0:0.2f}m from the rig".format(stand_off/1000)
print " - RMS deviation from a plane = {0:0.1f}mm".format(rms_error)

