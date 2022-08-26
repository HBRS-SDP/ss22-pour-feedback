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

def force_callback(data):
    global angle
    global step
    global p
    global traj
    global goal
    global cli
    global old_weight
    global finished
    
    # current iteration weight
    new_weight = int(data.data)
    
    # max allowed limit of change is 50 grams per cycle
    max_rate_of_change = 50
    
    # if the rate of change of weight is above a limit, ie. bulk pouring,
    if ((old_weight-new_weight) > max_rate_of_change) and (finished==False):
        # cancel all the current goal by vision callback
        cli.cancel_all_goals()
        # move back to an angle 3*step difference
        p.positions = [0.29,-0.42, 0.03,-1.07,angle-(3*step)]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Duration(1)
        traj.points = [p]
        goal.trajectory = traj

        # send message to the action server
        cli.send_goal(goal)
        # update the step to a lower value. Initially faster pouring, and this will decrease over time
        step*=0.95
        # waiting for result so that vision_callback wont be able to access it during that time
        cli.wait_for_result()

    # update the curr weight as prev weight
    old_weight = int(data.data)
    print("weight", new_weight)

def vision_callback(data):
    global angle
    global step
    global p
    global traj
    global goal
    global cli
    global level
    global old_weight
    global finished

    #rospy.loginfo('in callback')
    
    if int(data.data)<level:
        # in each iteration, move to the desired angle
        p.positions = [0.29,-0.42, 0.03,-1.07, angle]
        p.velocities = [0, 0, 0, 0, 0]
        # move it very fast as it needs to change in each iteration
        p.time_from_start = rospy.Duration(0.2)
        traj.points = [p]
        goal.trajectory = traj

        # send message to the action server
        cli.send_goal(goal)
        print("vision", data.data)
        # the angle is incremented to a new value defined by the step
        angle+=step
        cli.wait_for_result()

    else:
        # when limit is reached, cancel all goals and go back to initial state.
        finished = True
        rospy.sleep(0.2)
        cli.cancel_all_goals()
        p.positions = [0.29,-0.42, 0.03,-1.07,0.02]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Duration(3)
        traj.points = [p]
        goal.trajectory = traj

        # send message to the action server
        #cli.cancel_all_goals()
        cli.send_goal(goal)
        cli.wait_for_result()

        rospy.signal_shutdown("reached the target level")
        sys.exit(0)
    

if __name__ == '__main__':

    args = parse_args()
    # the level can be made as an argument and indicates to what level the pouring must be done.
    level = int(args.target)

    # angle variable stores the current roll angle
    angle = 0.02
    # step variable stores at which value, the angle increments in each iteration
    step = 0.02
    # old_weight is used to store the previous iteration weight to calculate the rate of change of weight
    old_weight = 0

    # Finish flag is used to stop the force callback
    finished = False


    rospy.init_node('pour_controller')

    # initialize action client
    cli = actionlib.SimpleActionClient(
        '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

    # wait for the action server to establish connection
    wait_result = cli.wait_for_server(rospy.Duration(5.))
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
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                        "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    
    # two subscribers are needed. One for vision level and weight.
    # since force feedback is faster, it can be used to update the global weight variable in the callback
    # vision feedback is slower and hence can be used as a limit checker for the arm turning
    # in the callback for force, when a condition happens, it can be used to turn back the arm by a bigger rad angle.
    rospy.Subscriber('/percent', Int32, vision_callback, queue_size=1)
    rospy.Subscriber('/grams', Float32, force_callback, queue_size=1)
    rospy.spin()
