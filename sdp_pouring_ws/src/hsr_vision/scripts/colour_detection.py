#!/usr/bin/env python3

from inspect import getclasstree
import math
from turtle import up
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np
import argparse


def nothing(x):
    pass

# Create windows
cv2.namedWindow('Track_bar')
cv2.namedWindow("Glass size")

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

# Set track bar for size of glass 
cv2.createTrackbar('Height', 'Glass size', 0, 370, nothing)
cv2.createTrackbar('Width', 'Glass size', 0, 400, nothing)

cv2.createTrackbar('Delta X', 'Glass size', -100, 100, nothing)
cv2.createTrackbar('Delta Y', 'Glass size', -50, 50, nothing)

cv2.setTrackbarPos('Height', 'Glass size', 100)
cv2.setTrackbarPos('Width', 'Glass size', 50)

cv2.setTrackbarPos('Delta X', 'Glass size', 0)
cv2.setTrackbarPos('Delta Y', 'Glass size', 0)


def parse_args():
  format_class = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser(formatter_class=format_class,
                  description='Default info')
  parser.add_argument('--target', help='Set the target percent filled in the glass')
  args = parser.parse_args(rospy.myargv()[1:])
  return args

def callback(data, target):
  
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
    
    # Set size of glass
    h_glass = cv2.getTrackbarPos('Height', 'Glass size')
    w_glass = cv2.getTrackbarPos('Width', 'Glass size')
	
    # Center point shift
    dx = cv2.getTrackbarPos('Delta X', 'Glass size')
    dy = cv2.getTrackbarPos('Delta Y', 'Glass size')
    center_x = 200+dx
    center_y = 370-dy
    
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
    
    # Get the target percentage
    percent_target = int(target)
    
    # focus on glass only with 2 pixels more in each direction #CHANGE HERE
    # mask_focus = mask[68:362,178:315]  # full glass
    interest = int(h_glass*percent_target/100)
    mask_x_left = center_x-int(w_glass)//2
    mask_x_right = center_x+int(w_glass)//2
    mask_focus = mask[center_y-interest-10:center_y+10,mask_x_left:mask_x_right]
    
    # cv2.rectangle(current_frame,(180,60),(320,370),(0,0,  255),2)   #bbox [(180,70),(320,360)], height glass ~= 290 
    cv2.circle(current_frame, (center_x,center_y), 5, (255,0,0), -1)
    cv2.rectangle(current_frame,(mask_x_left, center_y-int(h_glass)), (mask_x_right,center_y), (255,0,0),2)
    contours,_ = cv2.findContours(mask_focus.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    pub = rospy.Publisher("percent",Int32,queue_size=1)
    if len(contours)!=0:
        contour = max(contours, key = cv2.contourArea)

        x,y,w,h = cv2.boundingRect(contour)  
        #cv2.rectangle(current_frame,(x+mask_x_left,y+(center_y-interest)),(x+mask_x_left+w,y+(center_y-interest)+h),(0,255,0),2)
        cv2.rectangle(current_frame,(x+mask_x_left,y+(center_y-interest)),(x+mask_x_left+w,center_y),(0,255,0),2)
        
        percent = h/h_glass*100
              
        pub.publish(int(percent))
    else:
        pub.publish(0)
        
    cv2.imshow("camera", current_frame)
    cv2.imshow("HSV", result)
    #cv2.imshow("Mask Glass Only", mask_focus)
    
    wk = cv2.waitKey(1)
    if wk & 0xFF == ord('s'):
        print("save values")
        with open("./hsv_value.txt", "w") as f:
            f.writelines(str(lower))
            f.writelines("\n")
            f.writelines(str(upper))
    
    
#args = parse_args()
#target = args.target
target = 100

rospy.init_node('HSR_Vision')

# Node is subscribing to the video_frames topic
rospy.Subscriber('/hsrb/head_r_stereo_camera/image_raw', Image, callback, target)
# rospy.Subscriber('/hsrb/head_center_camera/image_raw', Image, callback)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
