#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

import sys
import os

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
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
_EXPLAIN1 = [u'グリッパの間に重さをはかりたいものを持ってきてください',
             u'Please set the object between my gripper']
_EXPLAIN2 = [u'グリッパを閉じます', u'I close my hand now']
_EXPLAIN3 = [u'これは{0}グラムです', u'Move me closer to the table so that i can start pouring']

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


if __name__ == '__main__':

    rospy.init_node('initial')

    # Set initial pose
    joint_controller = JointController()

    initial_position = JointState()
    initial_position.name.extend(['arm_lift_joint', 'arm_flex_joint',
                                  'arm_roll_joint', 'wrist_flex_joint',
                                  'wrist_roll_joint', 'head_pan_joint',
                                  'head_tilt_joint', 'hand_motor_joint'])
    initial_position.position.extend([0.29, -0.42, 0.03, -1.07,
                                       0.02, -0.10, -0.40, 1.2])
    joint_controller.move_to_joint_positions(initial_position)

    speaker = Speaker()
    speaker.speak_sentence(_EXPLAIN1[speaker.get_language()])
    rospy.sleep(2.0)

    speaker.speak_sentence(_EXPLAIN2[speaker.get_language()])
    rospy.sleep(1.0)
    
    joint_controller.grasp(-0.1)
    rospy.sleep(1.0)

    speaker.speak_sentence(_EXPLAIN3[speaker.get_language()])
    rospy.sleep(1.0)