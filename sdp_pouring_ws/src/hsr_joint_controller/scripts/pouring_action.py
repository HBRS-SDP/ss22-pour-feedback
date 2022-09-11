#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation

import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
from std_msgs.msg import Int32, Float32
import sys
import argparse


def parse_args():
  format_class = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser(formatter_class=format_class,
                  description='Default info')
  parser.add_argument('--target', help='Set the target percent to be filled in the glass')
  args = parser.parse_args(rospy.myargv()[1:])
  return args
  

class PouringAction(object):

    def __init__(self, level):
        # saves the target level
        self.level = level
        # angle variable stores the current roll angle. Initially its 0.02.
        self.angle = 0.02
        # step variable stores at which value the angle should be incremented in each iteration.
        self.step = 0.02
        # old_weight is used to store the previous iteration weight to calculate the rate of change of weight.
        self.old_weight = 0
        # Finish flag is used to stop the force callback by indicating that the target level is achieved.
        self.finished = False

        rospy.init_node('pour_controller')

        # initialize action client
        self.cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)

        # wait for the action server to establish connection
        wait_result = self.cli.wait_for_server(rospy.Duration(5.))
        if not wait_result:
            rospy.logerr('failed to contact action server after 5. seconds')
            sys.exit(0)

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
            '/hsrb/controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'arm_trajectory_controller' and c.state == 'running':
                    running = True

        # fill ROS message
        self.goal = control_msgs.msg.FollowJointTrajectoryGoal()
        self.traj = trajectory_msgs.msg.JointTrajectory()
        self.traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                            "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.p = trajectory_msgs.msg.JointTrajectoryPoint()
        
        # two subscribers are needed. One for vision and one for weight.
        # since force feedback is faster, it can be used to update the global weight variable in the callback.
        # vision feedback is slower and hence can be used as a limit checker for the arm turning
        # in the callback for force, when a condition happens, it can be used to turn back the arm by a bigger rad angle.
        rospy.Subscriber('/percent', Int32, self.vision_callback, queue_size=1)
        rospy.Subscriber('/grams', Float32, self.force_callback, queue_size=1)
        rospy.spin()

    def force_callback(self, force_data):
        # current iteration weight
        new_weight = int(force_data.data)
        
        # max allowed limit of change is 10 grams per cycle
        max_rate_of_change = 10
        
        # if the rate of change of weight is above a limit, ie. bulk pouring, and the level is not reached,
        if ((self.old_weight-new_weight) > max_rate_of_change) and (self.finished==False):
            # cancel all the current goal by vision callback
            self.cli.cancel_all_goals()
            # move back to an angle 10*current step size
            self.p.positions = [0.29,-0.42, 0.03,-1.07,self.angle-(10*self.step)]
            self.p.velocities = [0, 0, 0, 0, 0]
            self.p.time_from_start = rospy.Duration(1)
            self.traj.points = [self.p]
            self.goal.trajectory = self.traj

            # send message to the action server
            self.cli.send_goal(self.goal)
            # update the step to a lower value. Initially faster pouring, and this will decrease over time
            self.step*=0.95
            # waiting for result so that vision_callback wont be able to access it during that time
            self.cli.wait_for_result()

        # update the curr weight as prev weight
        self.old_weight = int(force_data.data)
        print("weight", new_weight, "grams")


    def vision_callback(self, vision_data):
        
        if (int(vision_data.data)<self.level) and (self.angle<2.3):
            # in each iteration, move to the desired angle
            self.p.positions = [0.29,-0.42, 0.03,-1.07, self.angle]
            self.p.velocities = [0, 0, 0, 0, 0]
            # move it very fast as it needs to change in each iteration
            self.p.time_from_start = rospy.Duration(0.2)
            self.traj.points = [self.p]
            self.goal.trajectory = self.traj

            # send message to the action server
            self.cli.send_goal(self.goal)
            print("vision", vision_data.data, "%")
            # the angle is incremented to a new value defined by the step
            self.angle+=self.step
            self.cli.wait_for_result()

        else:
            # when limit is reached, cancel all goals and go back to initial state.
            self.finished = True
            rospy.sleep(0.2)
            #self.cli.cancel_all_goals()
            self.p.positions = [0.29,-0.42, 0.03,-1.07,0.02]
            self.p.velocities = [0, 0, 0, 0, 0]
            self.p.time_from_start = rospy.Duration(3)
            self.traj.points = [self.p]
            self.goal.trajectory = self.traj

            # send message to the action server
            # cli.cancel_all_goals()
            self.cli.send_goal(self.goal)
            self.cli.wait_for_result()

            rospy.signal_shutdown("reached the target level")
            sys.exit(0)

    

if __name__ == '__main__':

    args = parse_args()
    # the level can be made as an argument and indicates to what level the pouring must be done.
    level = int(args.target)

    PouringAction(level)
