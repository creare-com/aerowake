# -*- coding: utf-8 -*-
"""
Created on Tue Apr 04 08:28:08 2017

@author: fiv
"""
import cv2
import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
# Plotting functions

def plot_original(left_img,right_img):    # plots two images side by side
    original = np.concatenate((left_img, right_img), axis=1)
    cv2.namedWindow('original',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('original', 1500,600)
    cv2.imshow("original", original)
    cv2.waitKey()
    
def detectedkpoints(image,kpoints,title='Detected keypoints'):      
    vmax = 80
    fig=plt.figure(title)
    plt.clf()
    ax1 = plt.subplot(111)
    ax1.imshow(image, cmap='gray', interpolation='nearest', vmax=vmax)
    ax1.axis('image')
    ax1.set_title(title)
    ax1.plot(kpoints[:,0], kpoints[:,1], '.')
    plt.show()

def matchedlines(left_img,right_img,left_kpoints,right_kpoints,title='Matched Keypoints'): 
    w1=2048; h1=1536
    w2=w1; h2=h1
    view = sp.zeros((max(h1, h2), w1 + w2), sp.uint8)
    view[:h1, :w1] = left_img  
    view[:h2, w1:] = right_img
    view[:, :] = view[:, :]  
    view[:, :] = view[:, :]
    for m in xrange(len(left_kpoints)):
        # draw the keypoints
        color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
        cv2.line(view, (int(left_kpoints[m,0]), int(left_kpoints[m,1])),(int(right_kpoints[m,0] + w1), int(right_kpoints[m,1])), color)
    cv2.namedWindow(title,cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 1500,600)
    cv2.imshow(title, view)
    cv2.waitKey()

def drawMatches_call(left_img,right_img,left_kpoints,right_kpoints,matches_sort,title='Matched Keypoints'): 
    # Draw first X matches.
    #images sorted in order of their distance
    #sel_matches_sort =sorted(matches, key = lambda x:x.distance)
    img = cv2.drawMatches(left_img,left_kpoints,right_img,right_kpoints,matches_sort,None,flags=2)
    #img = cv2.drawMatches(left_img,left_kpoints,right_img,right_kpoints,sel_matches_sort[:160],None,flags=2)
    cv2.namedWindow(title,cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 1500,600)
    cv2.imshow(title, img)
    cv2.waitKey()

def pointcloud(dpoints,title='3D Representation of Points from Stereo Reconstruction'): 
    fig = plt.figure('3D Plot of Points')
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(dpoints[:,0], dpoints[:,2], -dpoints[:,1], depthshade=False)
    #ax.scatter(points3d_filt[:,0], points3d_filt[:,2], -points3d_filt[:,1], c='r',marker='o')
    ax.set_xlabel('Horizontal')
    ax.set_ylabel('Depth')
    ax.set_zlabel('Vertical')
    ax.set_title(title)
    ax.set_xlim(-1500, 1500)
    ax.set_ylim(1000, 4000)
    ax.set_zlim(-1500, 1500)
    scaling = np.array(
        [getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3) 
    plt.show()
 
#this function plots an image and 2pairs of keypoints on top, made for left/right
#keypoints printed on top of item   
def detectedkpoints_2pair(image,kpoints1,kpoints2,title='Detected keypoints'):      
    vmax = 80
    plt.figure(title)
    plt.clf()
    ax1 = plt.subplot(111)
    ax1.imshow(image, cmap='gray', interpolation='nearest', vmax=vmax)
    ax1.axis('image')
    ax1.set_title(title)
    ax1.plot(kpoints1[:,0], kpoints1[:,1], '.',kpoints2[:,0], kpoints2[:,1], '.')
    leg = plt.legend(['Left', 'Right'],frameon=False, loc = 'best')
    for text in leg.get_texts():
        plt.setp(text, color = 'w')
    plt.show()
    
def video_subplot(image,kpoints1,kpoints2,points_3d,i,option=0):      
    vmax = 80
    fig=plt.figure(figsize=(12, 5.5))
    plt.clf()
    ax1 = plt.subplot(121)
    ax1.imshow(image, cmap='gray', interpolation='nearest', vmax=vmax)
    ax1.axis('image')
    ax1.set_title('Left Image')
    ax1.plot(kpoints1[:,0], kpoints1[:,1], '.',kpoints2[:,0], kpoints2[:,1], '.')
    leg = plt.legend(['Left', 'Right'],frameon=False, loc = 'best')
    for text in leg.get_texts():
        plt.setp(text, color = 'w')
    
    ax1 = fig.add_subplot(122, projection='3d')
    ax1.scatter(points_3d[:,0], points_3d[:,2], -points_3d[:,1], depthshade=False)
    ax1.set_xlabel('Horizontal')
    ax1.set_ylabel('Depth')
    ax1.set_zlabel('Vertical')
    ax1.set_title('3D Representation of Points from Stereo Reconstruction \n image %s'%str(i+1))
    ax1.set_xlim(-1500, 1500)
    ax1.set_ylim(1000, 4000)
    ax1.set_zlim(-1500, 1500)
    ax1.view_init(12.5, -60)
    fig.tight_layout()
    fig.canvas.draw()
    plt.show()
    
    if option:
        # Now we can save it to a numpy array.
        data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        return [data];
    