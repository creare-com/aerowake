# -*- coding: utf-8 -*-
"""
Created on Wed May 03 15:44:22 2017

@author: fiv
"""

import cv2
import os
from matplotlib import pyplot as plt
import numpy as np
import time
"""define image pair"""
#6755 left, 6575 right
CAL_FILE = 'stereo_ex2_cal.npz'

DATA_DIR = os.path.join('\\\\Olympus', 'Projects', '6598-Optical-Guide-II',
                        'Technical Work', 'Testing', 'Data',
                        '20170417 - Stereo tour of Creare','images_copy')
file_len=int(0.5*(len(os.walk(DATA_DIR).next()[2])-1))

# Load the required data
cal_data = np.load(CAL_FILE)
[height,width]=[1536,2048]

'''
left_maps = cv2.initUndistortRectifyMap(cal_data['cameraMatrix1'], cal_data['distCoeffs1'],
                                        cal_data['R1'], cal_data['P1'], (width, height), cv2.CV_16SC2)
right_maps = cv2.initUndistortRectifyMap(cal_data['cameraMatrix2'], cal_data['distCoeffs2'],
                                        cal_data['R2'], cal_data['P2'], (width, height), cv2.CV_16SC2)
'''

ii=400###defining image to analyze

LEFT_FILE='L '+'('+str(ii)+').tif'
RIGHT_FILE='R '+'('+str(ii)+').tif'
start_time = time.time()
left_img_orig = plt.imread(os.path.join(DATA_DIR, LEFT_FILE))#[:, :, 0]
right_img_orig = plt.imread(os.path.join(DATA_DIR, RIGHT_FILE))#[:, :, 0]

original = np.concatenate((left_img_orig, right_img_orig), axis=1)
cv2.namedWindow('original',cv2.WINDOW_NORMAL)
cv2.resizeWindow('original', 1500,600)
cv2.imshow("original", original)
cv2.waitKey(0)

'''#Was previously using initUndistortRectifyMap + remap, undistort combines both functions
left_img = cv2.remap(left_img_orig, left_maps[0], left_maps[1], cv2.INTER_LINEAR)# cv2.INTER_LANCZOS4)#, left_img_remap, cv2.BORDER_CONSTANT, 0)
right_img = cv2.remap(right_img_orig, right_maps[0], right_maps[1], cv2.INTER_LINEAR)# cv2.INTER_LANCZOS4)#LINEAR
'''
left_img = cv2.undistort(left_img_orig,
                        cal_data['cameraMatrix1'], cal_data['distCoeffs1'])
right_img = cv2.undistort(right_img_orig,
                        cal_data['cameraMatrix2'], cal_data['distCoeffs2'])
end_time=time.time()
process_time=(end_time-start_time)
print "----------------------------------------------"     
print("--- %s seconds ---" % process_time)   
'''#If filter wanted un comment
left_img = cv2.GaussianBlur(left_img, (3,3),3)
right_img = cv2.GaussianBlur(right_img, (3,3),3)
'''
rectified = np.concatenate((left_img, right_img), axis=1)
cv2.namedWindow('rectified',cv2.WINDOW_NORMAL)
cv2.resizeWindow('rectified', 1500,600)
cv2.imshow("rectified", rectified)
cv2.waitKey(0)

#Dropped stereo_2 lines by BJD
# Find epilines corresponding to points in right image (second image) and
# drawing its lines on left image
'''   ''' 
pts = np.array([[626, 534],[1100, 900],[400,300],[1500,1100],[870,675],
                    [1823, 1384]], dtype=np.float32)
lines = cv2.computeCorrespondEpilines(pts, 1, cal_data['F'])

plt.figure(num='Example Epi-Lines',figsize=(12, 5.5))
plt.subplot(121)
plt.imshow(left_img, cmap='gray', interpolation='nearest')
plt.axis('image')
plt.plot(pts[:, 0], pts[:, 1], 'rx')
'''plt.imshow(left_img, cmap='gray', interpolation='nearest')
plt_x = np.array([0, 2048])
for ln in lines:
    plt_y = (-ln[0, 2] - plt_x*ln[0, 0])/ln[0, 1]
    plt.plot(plt_x, plt_y, 'r-')
    plt.axis('image')'''
plt.subplot(122)
plt.imshow(right_img, cmap='gray', interpolation='nearest')
plt_x = np.array([0, 2048])
for ln in lines:
    plt_y = (-ln[0, 2] - plt_x*ln[0, 0])/ln[0, 1]
    plt.plot(plt_x, plt_y, 'r-')
    plt.axis('image')
plt.show()

#drawing epipolar lines following example
#here: http://docs.opencv.org/3.1.0/da/de9/tutorial_py_epipolar_geometry.html#gsc.tab=0
'''
def drawlines(img1,img2,lines,pts1,pts2):
   # img1 - image on which we draw the epilines for the points in img2
    #    lines - corresponding epilines
    r,c = img1.shape
    img1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
    img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv2.line(img1, (x0,y0), (x1,y1), color,1)
        img1 = cv2.circle(img1,tuple(pt1),5,color,-1)
        img2 = cv2.circle(img2,tuple(pt2),5,color,-1)
    return img1,img2
print pts1.shape

pts = np.array([[923, 761], [587, 418],
            [1292, 398], [1336, 1139],
            [560, 1146], [1314, 742],
            [558, 768], [919, 398],
            [934, 1142]], dtype=np.float32)

lines1 = cv2.computeCorrespondEpilines(pts.reshape(-1,1,2), 2,cal_data['F'])
lines1 = lines1.reshape(-1,3)
img5,img6 = drawlines(left_img,right_img,lines1,pts,pts)

# Find epilines corresponding to points in left image (first image) and
# drawing its lines on right image
lines2 = cv2.computeCorrespondEpilines(pts1.reshape(-1,1,2), 1,cal_data['F'])
lines2 = lines2.reshape(-1,3)
img3,img4 = drawlines(right_img,left_img,lines2,pts,pts)

plt.subplot(121),plt.imshow(img5)
plt.subplot(122),plt.imshow(img3)
plt.show()
'''