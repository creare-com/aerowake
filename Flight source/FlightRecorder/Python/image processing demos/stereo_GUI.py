# -*- coding: utf-8 -*-
"""
Created on Mon Jun 19 11:49:06 2017
https://books.google.com/books?id=iNlOCwAAQBAJ&pg=PA79&lpg=PA79&dq=recommended+opencv+stereosgbm+parameters&source=bl&ots=iS3Ji6Wsi8&sig=oWKoFas0gJeFv4XdXu4ccDX8IP4&hl=en&sa=X&ved=0ahUKEwipnfTmnsrUAhWBCD4KHa0iC2E4FBDoAQgjMAA#v=onepage&q=recommended%20opencv%20stereosgbm%20parameters&f=false
@author: fiv
First attempt. Accepts rectified images.  Some predefined parameters which can't be changes:
min_disp, num_disp, P1, P2, fullDP
5 trackbars are available for user.  Tried additional ones but were not available/reconized
in the "set" stereo functions
"""

import numpy as np
import cv2

def update(val=0):
    #disparity range is tuned for 'aloe' image pair 
    stereo.setBlockSize(cv2.getTrackbarPos('window_size','disparity'))
    stereo.setUniquenessRatio(cv2.getTrackbarPos('uniquenessRatio','disparity'))
    stereo.setSpeckleWindowSize(cv2.getTrackbarPos('speckleWindowSize','disparity'))
    stereo.setSpeckleRange(cv2.getTrackbarPos('speckleRange','disparity'))
    stereo.setDisp12MaxDiff(cv2.getTrackbarPos('disp12MaxDiff','disparity'))
    #stereo.set_preFilterCap(cv2.getTrackbarPos('preFilterCap','disparity'))
    #stereo.set_numberOfDisparities(cv2.getTrackbarPos('numberOfDisparities','disparity'))
    
    print 'computing disparity...'
    
    disp=stereo.compute(imgL,imgR).astype(np.float32)/16
    cv2.namedWindow('disparity',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('disparity', 750,600)
    cv2.imshow('disparity',(disp-min_disp)/numberOfDisparities)
    print 'disparity computed'

  
if __name__ == "__main__":
    window_size = 3
    min_disp = 0
    numberOfDisparities = 112-min_disp
    blockSize = window_size
    uniquenessRatio = 1
    speckleRange = 20
    speckleWindowSize = 100
    disp12MaxDiff = 20
    preFilterCap = 16
    P1 = 4*128
    P2 = 4*256
    fullDP = 1
    imgL = cv2.imread('L2_rectified.png')
    imgR = cv2.imread('R2_rectified.png')
    cv2.namedWindow('disparity',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('disparity', 750,600)
    cv2.createTrackbar('window_size', 'disparity', window_size, 21,update)
    #cv2.createTrackbar('numberOfDisparities', 'numberOfDisparities', numberOfDisparities, 15,update)
    cv2.createTrackbar('speckleRange', 'disparity', speckleRange, 50,update)
    cv2.createTrackbar('speckleWindowSize', 'disparity', speckleWindowSize, 200,update)
    cv2.createTrackbar('uniquenessRatio', 'disparity', uniquenessRatio, 50,update)
    cv2.createTrackbar('disp12MaxDiff', 'disparity', disp12MaxDiff, 250,update)
    #cv2.createTrackbar('preFilterCap', 'disparity', preFilterCap, 100,update)
    stereo=cv2.StereoSGBM_create(
        minDisparity = min_disp,
        numDisparities = numberOfDisparities,
        blockSize = window_size,
        uniquenessRatio = uniquenessRatio,preFilterCap=preFilterCap,
        speckleRange = speckleRange,
        speckleWindowSize = speckleWindowSize,
        disp12MaxDiff = disp12MaxDiff,
        #P1 = P1,
        #P2 = P2,
        mode=fullDP
    )
    update()
    cv2.namedWindow('left',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('left', 750,600)
    cv2.imshow('left',imgL)
    cv2.waitKey()
