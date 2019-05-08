# -*- coding: utf-8 -*-
"""
Created on Tue Nov 22 17:30:27 2016
sample code from: http://vgg.fiit.stuba.sk/2015-02/2783/
@author: fiv
in this file we will add feature detector, extractor and descriptor follwing 
"""
import cv2
import numpy as np
import scipy as sp
from matplotlib import pyplot as plt
#import draw_matches

"""options"""
print_file=0
point_cloud=0
thicker_line=0
print_rectified=0

"""define image pair"""
#6755 left, 6575 right
filename_R='C:/Users/fiv/Desktop/optical guide/Optical Guide II/6575/out6.png'
filename_L='C:/Users/fiv/Desktop/optical guide/Optical Guide II/6755/out6.png'
left_img = cv2.imread(filename_L,0)
right_img = cv2.imread(filename_R,0)
image_size = left_img.shape


'''
Compute rectification matrices: cv2.stereoRectify
Prepare undistortion maps for both cameras: cv2.initUndistortRectifyMap
Remap each image: cv2.remap
'''

#Calibration matrices
cameraMatrix1=np.array([[1.0579275e+3, 0e+0, 1.00822873e+3],
                [0.0e0, 1.05792754e+3, 7.71240073e2],
                [0.0e0, 0.0e0, 1.0e0]])
#print(cameraMatrix1)
cameraMatrix2=np.array([[1.05866022e+3, 0e+0, 1.00643122e+3],
                [0.0e0, 1.05866022e3, 7.32469703e2],
                [0.0e0, 0.0e0, 1.0e0]])
#print(cameraMatrix2)                
distCoeffs1=np.array([3.31221448e-1,-2.97438182e-2,7.71171658e-4,6.70278422e-4,
              2.36454130e-3,7.01276827e-1,0.0e0,0.0e0])
#print(distCoeffs1)
distCoeffs2=np.array([5.43666330e-1,-8.45702501e-2,6.86643912e-4,7.73000673e-4,
              1.29788999e-2,9.22401274e-1,0.0e0,0.0e0])
#print(distCoeffs2)
#imageSize=[2048,1536]
width=2048
height=1536

R=np.array([[0.99795388,-0.01079661,0.06301976],
   [0.01146327,0.999882,-0.01022662],
    [-0.06290191,0.01092811,0.99795988]])
T=np.array([[-221.8996694],[6.17156128],[31.20285697]])

rectify_scale = 0 # 0=full crop, 1=no crop
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (width, height), R, T, alpha = rectify_scale)

left_maps = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (width, height), cv2.CV_16SC2)
right_maps = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (width, height), cv2.CV_16SC2)
 
left_img_remap = cv2.remap(left_img, left_maps[0], left_maps[1], cv2.INTER_LANCZOS4)#, left_img_remap, cv2.BORDER_CONSTANT, 0)
right_img_remap = cv2.remap(right_img, right_maps[0], right_maps[1], cv2.INTER_LANCZOS4)#LINEAR

#saving image files
if print_rectified:
    cv2.imwrite('img_rect_out_6l.png',left_img_remap)
    cv2.imwrite('img_rect_out_6r.png',right_img_remap)
    '''cv2.imwrite('img_2_inside_5l.png',left_img)
    cv2.imwrite('img_2_inside_5r.png',right_img)'''


#displaying images
cv2.imshow("imageL", left_img);
cv2.imshow("imageR", right_img);
cv2.imshow("image1L", left_img_remap);
cv2.imshow("image2R", left_img_remap);
cv2.waitKey()
cv2.destroyAllWindows()

plt.imshow(left_img,'gray')
plt.show()
plt.imshow(left_img_remap,'gray')
plt.show()
plt.imshow(right_img,'gray')
plt.show()
plt.imshow(right_img_remap,'gray')
plt.show()

#result is dilated for marking the corners, not important
#dst = cv2.dilate(dst,None)
#stereo pairing using 
detector =  cv2.BRISK_create()
extractor = cv2.BRISK_create()
matcher = cv2.DescriptorMatcher_create("BruteForce")

'''For FAST/ORB combination uncomment this section, which replaces previous BRISK
cv2.ocl.setUseOpenCL(False)
left_kp = cv2.goodFeaturesToTrack(left_img_remap,150,0.01,10)
right_kp = cv2.goodFeaturesToTrack(right_img_remap,150,0.01,10)'''

left_kp = detector.detect(left_img_remap)
print left_kp
right_kp = detector.detect(right_img_remap)
print right_kp
l_kp, l_d = extractor.compute(left_img_remap, left_kp)
r_kp, r_d = extractor.compute(right_img_remap, right_kp)
matches = matcher.match(l_d, r_d)

dist = [m.distance for m in matches]    
thres_dist = (sum(dist) / len(dist)) * 0.75 #threshold could be varied
#or first detect everything then filter later
print thres_dist
# keep only the reasonable matches
sel_matches = [m for m in matches if m.distance < thres_dist]
print sel_matches
#another possible filtering alternative
#sel_matches = [m for m in matches if abs(l_kp[m.queryIdx].pt[1] - r_kp[m.trainIdx].pt[1]) &lt; 3]

# visualization of the matches
h1, w1 = left_img_remap.shape[:2]
h2, w2 = right_img_remap.shape[:2]
view = sp.zeros((max(h1, h2), w1 + w2), sp.uint8)
view[:h1, :w1] = left_img_remap  
view[:h2, w1:] = right_img_remap
view[:, :] = view[:, :]  
view[:, :] = view[:, :]

for m in sel_matches:
    # draw the keypoints
    # print m.queryIdx, m.trainIdx, m.distance
    color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
    cv2.line(view, (int(l_kp[m.queryIdx].pt[0]), int(l_kp[m.queryIdx].pt[1])) , (int(r_kp[m.trainIdx].pt[0] + w1), int(r_kp[m.trainIdx].pt[1])), color)

plt.imshow(view,'gray')
plt.show()

'''depth calculation'''
focal_length=1057.978466
baseline=16 #distance between cameras in centimeters
triangulation_constant=focal_length*baseline

plt.ion()
fig=plt.figure()
i=0
x=list()
depth=list()
disp_list=list()
for m in sel_matches:
        left_pt = l_kp[m.queryIdx].pt
        right_pt = r_kp[m.trainIdx].pt
        dispartity = abs(left_pt[0] - right_pt[0])
        disp_list.append(dispartity)
        #print dispartity        
        x.append(i)
        z = triangulation_constant / dispartity
        depth.append(z)        
        i=+1
        #print z
        '''
        plt.scatter(i,z)
        plt.show()
        plt.pause(0.0001)'''
#print depth
mean_disp = (sum(disp_list) / len(disp_list))
print mean_disp        

cv2.namedWindow('view',cv2.WINDOW_NORMAL)
cv2.resizeWindow('view', 1000,600)
cv2.imshow("view", view)
#cv2.waitKey()

plt.plot(depth)
plt.show()

# Draw first X matches.
#images sorted in order of their distance
sel_matches_sort =sorted(sel_matches, key = lambda x:x.distance)
img3 = cv2.drawMatches(left_img_remap,l_kp,right_img_remap,r_kp,sel_matches_sort[:160],None,flags=2)

cv2.namedWindow('view2',cv2.WINDOW_NORMAL)
cv2.resizeWindow('view2', 1000,600)
cv2.imshow("view2", img3)
cv2.waitKey()

#plt.imshow(img3),plt.show()

if print_file:
    #cv2.imwrite('window_1_brisk_matching.png',view)
    #cv2.imwrite('img_rect_window_5l.png',left_img_remap)
    #cv2.imwrite('img_rect_window_5r.png',right_img_remap)
    cv2.imwrite('sel_matches_office2.png',img3)
    
if point_cloud:
    xy = list()
    x = list()
    y = list()
    for m in sel_matches:
        xy=l_kp[m.queryIdx].pt
        x_iter=xy[0]
        y_iter=xy[1]
        x.append(x_iter)
        y.append(y_iter)
    #xyz=x+y+depth
    xyz = np.column_stack((x,y,depth))
    #print xyz
    f = open('point_cloud_office_1.txt', 'w') # open for 'w'riting
    # Loop through each item in the list
    # and write it to the output file.
    for eachitem in xyz:
        f.write(str(xyz))        
        #f.write(str(eachitem)+'\n')

        # Close the output file
    f.close()
    print np.array_str(xyz)

