# Copyright (C) 2016 Toyota Motor Corporation

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image


def create_mask_image(channel, th_max, th_min):
    ret, src = cv2.threshold(channel, th_max, 255, cv2.THRESH_TOZERO_INV)
    ret, dst = cv2.threshold(src, th_min, 255, cv2.THRESH_BINARY)
    return dst


class ColorDetection(object):
    """color detection class using OpenCV"""

    H_LOW_THRESHOLD = 0
    H_HIGH_THRESHOLD = 15
    S_LOW_THRESHOLD = 50
    S_HIGH_THRESHOLD = 255
    V_LOW_THRESHOLD = 50
    V_HIGH_THRESHOLD = 255

    def __init__(self):
        topic_name = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        self._bridge = CvBridge()
        self._input_image = None

        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber(
            topic_name, Image, self._color_image_cb)
        # Wait until connection
        rospy.wait_for_message(topic_name, Image, timeout=5.0)

    def _color_image_cb(self, data):
        try:
            self._input_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def extract_color(self):
        # BGR -> HSV
        hsv = cv2.cvtColor(self._input_image, cv2.COLOR_BGR2HSV)

        # Break down channels
        h, s, v = cv2.split(hsv)

        # Create mask images(H, S, V)
        h_dst = create_mask_image(h, self.H_HIGH_THRESHOLD,
                                  self.H_LOW_THRESHOLD)
        s_dst = create_mask_image(s, self.S_HIGH_THRESHOLD,
                                  self.S_LOW_THRESHOLD)
        v_dst = create_mask_image(v, self.V_HIGH_THRESHOLD,
                                  self.V_LOW_THRESHOLD)

        dst = cv2.bitwise_and(h_dst, s_dst)
        dst = cv2.bitwise_and(dst, v_dst)

        # Below part is edited
        # grayImage = cv2.cvtColor(originalImage, cv2.COLOR_BGR2GRAY)

        return hsv


def main():
    rospy.init_node('hsrb_color_detection')
    try:
        color_detection = ColorDetection()
        spin_rate = rospy.Rate(30)

        # UpdateGUI Window
        while not rospy.is_shutdown():
            dst_image = color_detection.extract_color()
            cv2.imshow("Color Detection Image Window", dst_image)
            cv2.waitKey(3)
            spin_rate.sleep()

    except rospy.ROSException as wait_for_msg_exception:
        rospy.logerr(wait_for_msg_exception)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
