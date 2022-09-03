#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation
"""Speak Object Weight Sample"""

import math
import matplotlib.pyplot as plt
import os
import sys
from std_msgs.msg import Float32
import actionlib
import numpy as np
from scipy import signal

import time 
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
import tf
import geometry_msgs.msg
from  tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion

#from geometry_msgs.msg import compensated 
import rospy
from sensor_msgs.msg import JointState
from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)
from tmc_msgs.msg import (
    TalkRequestAction,
    TalkRequestGoal,
    Voice
)


_CONNECTION_TIMEOUT = 10.0

# Create speak sentences
# Index 0: in Japanese, 1: in English

_EXPLAIN4 = [u'hdhgh', u'Please move away as I get to my initial position']

_EXPLAIN3 = [u'jskjks',u'Hello this is team pouring']
_EXPLAIN1 = [u'グリッパの間に重さをはかりたいものを持ってきてください',
             u'Please set the object between my gripper']
_EXPLAIN2 = [u'グリッパを閉じます', u'I close my hand now']
_ANSWER = [u'これは{0}グラムです', u'This is {0} gram']


def compute_difference(pre_data_list, post_data_list,initial,post):
    if (len(pre_data_list) != len(post_data_list)):
        raise ValueError('Argument lists differ in length')
    # Calcurate square sum of difference

    angle=np.degrees(initial)
    x_new=post_data_list[0]*math.cos(angle)-post_data_list[1]*math.sin(angle)
    y_new=post_data_list[0]*math.sin(angle)+post_data_list[1]*math.cos(angle)
    z_new=post_data_list[2]

    x_old=pre_data_list[0]
    y_old=pre_data_list[1]
    z_old=pre_data_list[2]

    result=math.sqrt(math.pow(x_old-x_new,2)+math.pow(y_old-y_new,2)+math.pow(z_old - z_new,2))
    return result

class ForceSensorCapture(object):
    """Subscribe and hold force sensor data"""

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0
        self.wrist_roll_initial_pos=0.0#rospy.Subscriber('/hsrb/joint_states/position[-1]',Float32)
        self.wrist_roll_post_pos=0.0
        # Subscribe force torque sensor data from HSRB

        #ft_sensor_topic = '/hsrb/wrist_wrench/raw'
        
        #compensation_node/biquad_lowpass_filter/55.0
        # Here we are using raw but we can use compensated instead
        ft_sensor_topic = '/hsrb/wrist_wrench/compensated'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)
        self.wrist_roll_sub=rospy.Subscriber('/hsrb/joint_states',JointState,self.__angle_cb)

        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)
    
    def __angle_cb(self,data):
        self.wrist_roll_angle=data.position[-1]


    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]
    # Added separately
    def get_current_angle(self):
        return self.wrist_roll_angle

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z
        #self.wrist_roll_post_pos=rospy.Subscriber('/hsrb/joint_states/position[-1]',Float32)
        
        # Applying low pass filter for force values

        FX = []
        FY = []
        FZ = []

        FX.append(self._force_data_x)
        FY.append(self._force_data_y)
        FZ.append(self._force_data_z)

        fs = 124.95 
        fc = 55
        # 55

        w = fc / (fs/2)

        b,a = signal.butter(5,w,'low')
        # 5
        self._force_data_x = signal.lfilter(b, a, FX) #Forward filter
        self._force_data_y = signal.lfilter(b, a, FY)
        self._force_data_z = signal.lfilter(b, a, FZ)

        # Reference: https://answers.ros.org/question/312833/how-do-i-implement-a-low-pass-filter-to-reduce-the-noise-coming-from-a-topic-that-is-publishing-a-wrenchstamped-msg-type-using-a-python-script/




class JointController(object):
    """Control arm and gripper"""

    def __init__(self):
        joint_control_service = '/safe_pose_changer/change_joint'
        grasp_action = '/hsrb/gripper_controller/grasp'
        self._joint_control_client = rospy.ServiceProxy(
            joint_control_service, SafeJointChange)

        self._gripper_control_client = actionlib.SimpleActionClient(
            grasp_action, GripperApplyEffortAction)

        # Wait for connection
        try:
            self._joint_control_client.wait_for_service(
                timeout=_CONNECTION_TIMEOUT)
            if not self._gripper_control_client.wait_for_server(rospy.Duration(
                    _CONNECTION_TIMEOUT)):
                raise Exception(grasp_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def move_to_joint_positions(self, goal_joint_states):
        """Joint position control"""
        try:
            req = SafeJointChangeRequest(goal_joint_states)
            res = self._joint_control_client(req)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        return res.success
   


    def grasp(self, effort):
        """Gripper torque control"""
        goal = GripperApplyEffortGoal()
        goal.effort = effort

        # Send message to the action server
        if (self._gripper_control_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False


class Speaker(object):
    """Speak sentence in robot's language"""

    def __init__(self):
        talk_action = '/talk_request_action'
        self._talk_request_client = actionlib.SimpleActionClient(
            talk_action, TalkRequestAction)

        # Wait for connection
        try:
            if not self._talk_request_client.wait_for_server(
                    rospy.Duration(_CONNECTION_TIMEOUT)):
                raise Exception(talk_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        # Detect robot's language
        if os.environ['LANG'] == 'ja_JP.UTF-8':
            self._lang = Voice.kJapanese
        else:
            self._lang = Voice.kEnglish

    def get_language(self):
        return self._lang

    def speak_sentence(self, sentence):
        goal = TalkRequestGoal()
        goal.data.language = self._lang
        goal.data.sentence = sentence

        if (self._talk_request_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False


def force_to_grams():
    pub = rospy.Publisher('grams',Float32, queue_size = 10)
    rate = rospy.Rate(5)
    rospy.init_node('hsrb_speak_object_weight')
    # Start force sensor capture
    force_sensor_capture = ForceSensorCapture()

    # Asking robot to introduce
    speaker = Speaker()
    speaker.speak_sentence(_EXPLAIN3[speaker.get_language()])
    rospy.sleep(2.0)
    
    # Asking robot to initialize position
    speaker.speak_sentence(_EXPLAIN4[speaker.get_language()])
    rospy.sleep(2.0)

    # Set initial pose
    joint_controller = JointController()

    initial_position = JointState()
    initial_position.name.extend(['arm_lift_joint', 'arm_flex_joint',
                                  'arm_roll_joint', 'wrist_flex_joint',
                                  'wrist_roll_joint', 'head_pan_joint',
                                  'head_tilt_joint', 'hand_motor_joint'])
    initial_position.position.extend([0.29, -0.42, 0.03, -1.07,
                                      0.02, -0.10, -0.40, 0.60])
    
    joint_controller.move_to_joint_positions(initial_position)

    # Get initial data of force sensor
    pre_force_list = force_sensor_capture.get_current_force()
    
    pre_angle=force_sensor_capture.get_current_angle()

    # Ask user to set object
    speaker.speak_sentence(_EXPLAIN1[speaker.get_language()])
    rospy.sleep(2.0)

    # Inform user of next gripper action
    speaker.speak_sentence(_EXPLAIN2[speaker.get_language()])
    rospy.sleep(1.0)

    # Grasp the object
    joint_controller.grasp(-0.1)

    # Wait until force sensor data become stable
    rospy.sleep(1.0)
    post_force_list = force_sensor_capture.get_current_force()

    post_angle=force_sensor_capture.get_current_angle()

    #initial_position_angle=force_sensor_capture.wrist_roll_post_pos
    #post_position_angle=force_sensor_capture.wrist_roll_initial_pos

    force_difference = compute_difference(pre_force_list, post_force_list,pre_angle,post_angle)

    # Convert newton to gram
    weight = round(force_difference / 9.81 * 1000, 1)
    # Speak object weight in first decimal place
    speaker.speak_sentence(_ANSWER[speaker.get_language()].format(weight))
   
    # publishing weight while the turning action takes place
    pub = rospy.Publisher( 'grams', Float32, queue_size=1)


    weights = []
    rospy.Rate(10)
    while not rospy.is_shutdown():


        # Getting new force sensor reading
        post_force_list = force_sensor_capture.get_current_force()

        post_angle=force_sensor_capture.get_current_angle()


        # Computing difference from initial and new force sensor readings
        #initial_position_angle=force_sensor_capture.wrist_roll_post_pos
        #post_position_angle=force_sensor_capture.wrist_roll_initial_pos
        force_difference = compute_difference(pre_force_list, post_force_list,pre_angle,post_angle)
        # Convert newton to gram
        weight = round(force_difference / 9.81 * 1000, 1)
        weights.append(weight)

        # Applying median filtering to weights with window size
        x = 5
        if len(weights)>5:
            median = np.median(weights[:x])
            rospy.loginfo(weight)
            pub.publish(weight)
            rospy.sleep(0.1)
            x+=5
        else:
            continue
        #print(weight)
        #print(post_force_list) 
        

if __name__ == '__main__':
    rospy.init_node('hsrb_speak_object_weight')
    force_to_grams()
    rospy.spin()


