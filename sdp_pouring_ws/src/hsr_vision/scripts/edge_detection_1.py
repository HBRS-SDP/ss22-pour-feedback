#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
"""Flesh Color Detection Sample"""
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
import numpy as np


reference_frame = None
prev_level = 0.0
flag = False


class PourDetection(object):

    def __init__(self):
        # RGBD camera
        # topic_name = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        # Stereo right camera
        topic_name = '/hsrb/head_r_stereo_camera/image_raw'
        self._bridge = CvBridge()
        self._input_image = None

        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber(topic_name, Image, self._color_image_cb)
        # Wait until connection
        rospy.wait_for_message(topic_name, Image, timeout=5.0)
        # image is displayed only initially to get the cordinates
        plt.imshow(self._input_image)
        plt.show()

    def _color_image_cb(self, data):
        try:
            self._input_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            # adjust the cordinates as per the initilializer visualizations step
            self._input_image = self._input_image[449:875, 460:688]
            # self._input_image = self._input_image[275:500, 275:475]
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def extract_level(self):
        global flag
        global reference_frame
        global prev_level
        gray = cv2.cvtColor(self._input_image, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray,(7,7),cv2.BORDER_DEFAULT)

        ksize = 3
        #gX = cv2.Sobel(gray_blurred, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=ksize)
        gY = cv2.Sobel(gray_blurred, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=ksize)
        if flag == False:
            reference_frame = np.copy(gY)
            flag = True    
        gY_new = gY - reference_frame
        #gX = cv2.convertScaleAbs(gX)
        gY_new = cv2.convertScaleAbs(gY_new)
        gY = cv2.convertScaleAbs(gY)

        #combined = cv2.addWeighted(gX, 0.5, gY, 0.5, 0)
        
        # combined can be used if necessary
        
        # apply gaussian blur to remove noise
        blurred_img = cv2.GaussianBlur(gY_new,(7,7),cv2.BORDER_DEFAULT)
        
        # apply thresholding to 0 and 255
        thrshld_img = np.copy(blurred_img)

        # threshold level is set to 10 to remove the noise
        threshold = 20
        thrshld_img = np.where(thrshld_img > threshold, 255, 0)
        thrshld_img = thrshld_img.astype('uint8')

        # scailing down the image for easier computation
        # percent by which the image is resized
        scale_percent = 10

        # new width and height
        width = int(thrshld_img.shape[1] * scale_percent / 100)
        height = int(thrshld_img.shape[0] * scale_percent / 100)

        # dsize
        dsize = (width, height)

        # resize image, the interpolation parameter can be changed.
        resized_img = cv2.resize(thrshld_img, dsize, interpolation=cv2.INTER_AREA)

        
        # create an empty array of same size as that of original image.
        # image into 10 equal blocks
        # once 25% or 50% of that block is white, the entire block can be considered to be white
        
        block_img = np.zeros_like(resized_img)

        # calculate the number of white pixels in each row
        cnt_white = np.count_nonzero(resized_img, axis=1)
        # rows which can be considered as lines
        lines = np.where(cnt_white > int((width-(0.2*width))/2))[0]
        
        # our required line index needs to be identified
        # and make all pixels under that line as white

        # also need to add condition to check current level is higher than previous level
        idx = 1
        if len(lines)>idx:
            block_img[lines[idx]:] = 255
            level = int(((height - lines[idx]+1)/height)*100)
        else:
            level = 0

        if level < prev_level:
            level = prev_level

        print("level is", level)

        prev_level = level

        pub = rospy.Publisher("percent",Int32,queue_size=1)
        pub.publish(level)
        
        return self._input_image, gY_new, thrshld_img, block_img



def main():
    rospy.init_node('hsrb_pouring_level_detection')
    try:
        pouring_detection = PourDetection()
        spin_rate = rospy.Rate(30)

        # UpdateGUI Window
        while not rospy.is_shutdown():
            img, gY, thrshld_img, blk_img = pouring_detection.extract_level()
            cv2.imshow("Original Image", img) 
            cv2.imshow("Gradient Y Image", gY)
            cv2.imshow("Threshold Image", thrshld_img)
            blk_img = cv2.resize(blk_img, (200, 480), interpolation=cv2.INTER_AREA)
            cv2.imshow("Result Image", blk_img)
            cv2.waitKey(3)
            spin_rate.sleep()


    except rospy.ROSException as wait_for_msg_exception:
        rospy.logerr(wait_for_msg_exception)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
