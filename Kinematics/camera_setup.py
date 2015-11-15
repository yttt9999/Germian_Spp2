__author__ = 'kevin'
import flycapture2 as fc2
import cv2
import numpy as np
#Define camera matrix and distortion coefficient
camera_M = np.array([[411.42948014, 0, 331.38683566], [0, 411.0871804, 208.84949458], [0, 0, 1]])
distortion_coefficient = np.array([-0.33504357,0.07164384, -0.00376871 ,0.0016508  ,0.15057044])
reference_4_points = [np.array([[236,153]]),np.array([[466,161]]),np.array([[514,411]]),np.array([[132,371]])]

#Define Gaussian blur parameters:
Gaussian_Ratio = 5
Gaussian_Ratio_dark = 7

#define parameters for crop
crop_x = 320
crop_y_up = 240
crop_y_down = 240
def flycamera_init():
    flycamera = fc2.Context()
    flycamera.connect(*flycamera.get_camera_from_index(0))
    #set up the camera frame rate and mode
    flycamera.set_video_mode_and_frame_rate(fc2.VIDEOMODE_640x480Y8,fc2.FRAMERATE_60)
    #set up the camera exposure
    p = flycamera.get_property(fc2.AUTO_EXPOSURE)
    p['auto_manual_mode'] = False
    p['value_a'] = 25 #Usually I will do 25
    flycamera.set_property(**p)
    return flycamera

def undistort_image(flycamera):
    im = fc2.Image()
    flycamera.retrieve_buffer(im)
    gray_image = np.array(im)
    image = cv2.cvtColor(gray_image,cv2.COLOR_GRAY2BGR)
    #Undistort the Image
    h,w = image.shape[:2]
    #mask is saved to crop the image
    mask = np.zeros(image.shape[:2],dtype = "uint8")
    cx = image.shape[1]/2
    cy = image.shape[0]/2
    cv2.rectangle(mask,(cx-crop_x,cy-crop_y_up),(cx+crop_x,cy+crop_y_down),255,-1)

    #Undistort the original image
    newcamera, roi = cv2.getOptimalNewCameraMatrix(camera_M, distortion_coefficient, (w,h), 0)
    fixed_image = cv2.undistort(image,camera_M,distortion_coefficient,None,newcamera)
    #crop the undistorted image
    fixed_image = cv2.bitwise_and(fixed_image,fixed_image,mask = mask)
    return fixed_image

def find_brightest_point(image):
    gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    #Apply Gaussian Blur
    gray_image = cv2.GaussianBlur(gray_image,(Gaussian_Ratio,Gaussian_Ratio),0)
    (minVal, maxVal, minLoc,maxLoc) = cv2.minMaxLoc(gray_image)
    return maxVal,maxLoc

def find_darkest_point(image):
    gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    #Apply Gaussian Blur
    gray_image = cv2.GaussianBlur(gray_image,(Gaussian_Ratio_dark,Gaussian_Ratio_dark),0)
    (minVal, maxVal, minLoc,maxLoc) = cv2.minMaxLoc(gray_image)
    return minVal,minLoc

def four_pts_transormation(left_image,left_p):
    left_pts = np.zeros((4,2),dtype = "float32")
    j = 0
    for i in left_p:
        left_pts[j] = i
        j += 1
    W1 = np.sqrt((left_pts[1,0]-left_pts[0,0])**2+(left_pts[1,1]-left_pts[0,1])**2)
    W2 = np.sqrt((left_pts[2,0]-left_pts[3,0])**2+(left_pts[2,1]-left_pts[3,1])**2)
    max_length = max(int(W1),int(W2))
    H1 = np.sqrt((left_pts[1,0]-left_pts[2,0])**2+(left_pts[1,1]-left_pts[2,1])**2)
    H2 = np.sqrt((left_pts[0,0]-left_pts[3,0])**2+(left_pts[0,1]-left_pts[3,1])**2)
    max_height = max(int(H1),int(H2))
    right_pts = np.array([[0,0],
                          [max_length-1,0],
                          [max_length-1,max_height-1],
                          [0,max_height-1]],dtype = "float32")
    H = cv2.getPerspectiveTransform(left_pts,right_pts)
    new_image = cv2.warpPerspective(left_image,H,(max_length,max_height))
    return new_image,H

