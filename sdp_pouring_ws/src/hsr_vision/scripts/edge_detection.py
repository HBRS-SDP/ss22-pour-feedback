#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np



class PourDetection(object):

    def __init__(self):
        # Stereo right camera
        topic_name = '/hsrb/head_r_stereo_camera/image_raw'
        self._bridge = CvBridge()
        # used to store the input image
        self._input_image = None
        # use to store the initial reference frame for background subtraction
        self.reference_frame = None
        # self.prev_level = 0.0
        # used to make notify whether the initial reference frame is saved.
        self.ref_flag = False
        # used to store the bounding box points selected by user
        self.points = []
        # Flag used to notify that the bounding box is selected temporarily
        self.init_flag = False
        # flag used to notify that the bounding box is successfuly created
        self.initialization = False

        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber(topic_name, Image, self._color_image_cb)
        # Wait until connection
        rospy.wait_for_message(topic_name, Image, timeout=5.0)
        # calling the bounding box generator function
        self.boundingBox()
        

    def mouseCallback(self, event, x, y, flags, param):
        # mouse callback function used while bounding box generation
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points = [(x, y), (x, y)]
        if event == cv2.EVENT_LBUTTONUP:
            self.points[1] = (x, y)
            self.init_flag = True
        elif event == cv2.EVENT_MOUSEMOVE and (flags == cv2.EVENT_FLAG_LBUTTON):
            self.points[1] = (x, y)

    def boundingBox(self):

        new_image = self._input_image.copy()
        prev_point = (0,0)
        cv2.namedWindow("Bounding box Selector")
        cv2.setMouseCallback("Bounding box Selector", self.mouseCallback)

        while True:
            while self.init_flag==False:
                cv2.imshow("Bounding box Selector", new_image)
                key = cv2.waitKey(1)
                if len(self.points)==2:
                    if self.points[1]!=prev_point:
                        new_image = self._input_image.copy()
                        # draw a rectangle using the mouse down and up points
                        cv2.rectangle(new_image, self.points[0], self.points[1], (255, 0, 0), 2)
                        prev_point = self.points[1]

            cv2.imshow("Bounding box Selector", new_image)
            key = cv2.waitKey(1) & 0xFF
            # reset the bounding box
            if key == ord("r"):
                new_image = self._input_image.copy()
                self.init_flag = False
            # confirm the bounding box
            elif key == ord("c"):
                break
        cv2.destroyWindow("Bounding box Selector")
        # to indicate that the bounding box has been successfully generated.
        self.initialization = True


    def _color_image_cb(self, data):
        try:
            self._input_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            # adjust the cordinates as per the initilializer visualizations step
            # only after completing initialization
            if self.initialization==True and (self.points[0]!=self.points[1]):
                self._input_image = self._input_image[self.points[0][1]:self.points[1][1],self.points[0][0]:self.points[1][0]]

        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def extract_level(self):
        
        gray = cv2.cvtColor(self._input_image, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray,(7,7),cv2.BORDER_DEFAULT)

        ksize = 3
        # change dy to 2 also
        gY = cv2.Sobel(gray_blurred, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=ksize)
        if self.ref_flag == False:
            self.reference_frame = np.copy(gY)
            self.ref_flag = True

        gY_new = gY - self.reference_frame
        gY_new = cv2.convertScaleAbs(gY_new)
        gY = cv2.convertScaleAbs(gY)
        
        # combined can be used if necessary
        
        # apply gaussian blur to remove noise
        blurred_img = cv2.GaussianBlur(gY_new,(7,7),cv2.BORDER_DEFAULT)
        
        # apply thresholding to 0 and 255
        thrshld_img = np.copy(blurred_img)

        # threshold level is set to 20 to remove the noise
        threshold = 20
        thrshld_img = np.where(thrshld_img > threshold, 255, 0)

        if self.ref_flag == False:
            self.reference_pixels = np.where(thrshld_img == 255)
            self.ref_flag = True

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
            level = self.prev_level


        print("level is", level)

        self.prev_level = level

        pub = rospy.Publisher("percent",Int32,queue_size=1)
        pub.publish(level)
        
        return self._input_image, gY_new, thrshld_img, block_img



def main():
    rospy.init_node('hsrb_pouring_level_detection')
    try:
        pouring_detection = PourDetection()
        spin_rate = rospy.Rate(10)
        rospy.sleep(0.5)
        
        # UpdateGUI Window
        # check this self.points[0]!=self.points[1] here also maybe. call extract level only when self._image is of smaller size
        while (not rospy.is_shutdown()) and (pouring_detection.initialization==True):
            spin_rate.sleep()
            img, gY, thrshld_img, blk_img = pouring_detection.extract_level()
            cv2.imshow("Original Image", img) 
            cv2.imshow("Gradient Y Image", gY)
            cv2.imshow("Threshold Image", thrshld_img)
            blk_img = cv2.resize(blk_img, (pouring_detection._input_image.shape[1], pouring_detection._input_image.shape[0]), interpolation=cv2.INTER_AREA)
            cv2.imshow("Result Image", blk_img)
            cv2.waitKey(3)
            # spin_rate.sleep()


    except rospy.ROSException as wait_for_msg_exception:
        rospy.logerr(wait_for_msg_exception)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
