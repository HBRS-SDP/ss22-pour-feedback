#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image
import numpy as np

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
cv2.setTrackbarPos('HMax', 'Track_bar', 179)
cv2.setTrackbarPos('SMax', 'Track_bar', 255)
cv2.setTrackbarPos('VMax', 'Track_bar', 255)


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
    current_frame = cv2.cvtColor(br.imgmsg_to_cv2(data), cv2.COLOR_RGB2BGR)
    
    # Convert to HSV format and color threshold
    hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(current_frame, current_frame, mask=mask)

    
    cv2.rectangle(current_frame,(560,430),(685,670),(0,0,255),2)    #y: 321->570
    cv2.imshow("camera", current_frame)
    cv2.imshow("HSV", result)
    cv2.waitKey(1)

rospy.init_node('Baxter_Image')

# Node is subscribing to the video_frames topic
rospy.Subscriber('/hsrb/head_r_stereo_camera/image_raw', Image, callback)
# rospy.Subscriber('/hsrb/head_center_camera/image_raw', Image, callback)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
