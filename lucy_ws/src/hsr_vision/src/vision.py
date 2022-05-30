#!/usr/bin/env python3

import math
from turtle import up
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np
import imutils

def nothing(x):
    pass

# Create a window
cv2.namedWindow('Track_bar')

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('HMin', 'Track_bar', 0, 179, nothing)
cv2.createTrackbar('SMin', 'Track_bar', 0, 255, nothing)
cv2.createTrackbar('VMin', 'Track_bar', 0, 255, nothing)
cv2.createTrackbar('HMax', 'Track_bar', 0, 179, nothing)
cv2.createTrackbar('SMax', 'Track_bar', 0, 255, nothing)
cv2.createTrackbar('VMax', 'Track_bar', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMin', 'Track_bar', 8)
cv2.setTrackbarPos('SMin', 'Track_bar', 131)
cv2.setTrackbarPos('VMin', 'Track_bar', 43)
cv2.setTrackbarPos('HMax', 'Track_bar', 41)
cv2.setTrackbarPos('SMax', 'Track_bar', 241)
cv2.setTrackbarPos('VMax', 'Track_bar', 235)


def callback(data):
  
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin', 'Track_bar')
    sMin = cv2.getTrackbarPos('SMin', 'Track_bar')
    vMin = cv2.getTrackbarPos('VMin', 'Track_bar')
    hMax = cv2.getTrackbarPos('HMax', 'Track_bar')
    sMax = cv2.getTrackbarPos('SMax', 'Track_bar')
    vMax = cv2.getTrackbarPos('VMax', 'Track_bar')

    # Set minimum and maximum HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])
    
	
    # Convert ROS Image message to OpenCV image
    ori_frame = cv2.cvtColor(br.imgmsg_to_cv2(data), cv2.COLOR_RGB2BGR)
    current_frame = ori_frame[300:1000,300:1000]
    current_frame = cv2.blur(current_frame,(5,5))
    
    # Convert to HSV format and color threshold
    hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(current_frame, current_frame, mask=mask)

    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask_focus = mask[75:375,235:390]  #focus on glass only with 2 pixels more in each direction
    
    cv2.rectangle(current_frame,(240,80),(385,370),(0,0,255),2)    #bbox [(260,130),385,370)], height glass ~= 290
    
    contours,_ = cv2.findContours(mask_focus.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    pub = rospy.Publisher("percent",Int32)
    if len(contours)!=0:
        contour = max(contours, key = cv2.contourArea)

        x,y,w,h = cv2.boundingRect(contour)  
        cv2.rectangle(current_frame,(x+235,y+75),(x+235+w,y+75+h),(0,255,0),2)
        
        percent = math.ceil(h/290*100)
        pub.publish(percent)
    else:
        pub.publish(0)
        
    cv2.imshow("camera", current_frame)
    cv2.imshow("HSV", result)
    cv2.imshow("Mask Glass Only", mask_focus)
    
    # cv2.imshow("Test", mask[130:371,260:386])
    # rospy.Publisher("std_msgs/Float64",Float64)
    
    wk = cv2.waitKey(1)
    if wk & 0xFF == ord('s'):
        print("save values")
        with open("./hsv_value.txt", "w") as f:
            f.writelines(str(lower))
            f.writelines("\n")
            f.writelines(str(upper))
    
    
rospy.init_node('HSR_Vision')

# Node is subscribing to the video_frames topic
rospy.Subscriber('/hsrb/head_r_stereo_camera/image_raw', Image, callback)
# rospy.Subscriber('/hsrb/head_center_camera/image_raw', Image, callback)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
