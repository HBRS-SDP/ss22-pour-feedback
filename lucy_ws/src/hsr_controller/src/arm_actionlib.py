#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg


rospy.init_node('test')

# initialize action client
cli = actionlib.SimpleActionClient(
    '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
    control_msgs.msg.FollowJointTrajectoryAction)

# wait for the action server to establish connection
cli.wait_for_server()

print("after server")

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
print("I am line 36")
p = trajectory_msgs.msg.JointTrajectoryPoint()

# initial position values
p.positions = [0.29,-0.42, 0.03,-1.07,0.02]
p.velocities = [0, 0, 0, 0, 0]

x = 0.02
y = -5.0

# code for constant +3 movement
for _ in range(20):
    p.positions = [0.29,-0.42, 0.03,-1.07,x]
    p.velocities = [0, 0, 0, 0, 0]
    p.time_from_start = rospy.Duration(3)
    traj.points = [p]
    goal.trajectory = traj

    # send message to the action server
    cli.send_goal(goal)
    y = 3.0

    x=x+0.2

    # wait for the action server to complete the order
    cli.wait_for_result()

    if x > 3.5:
        cli.cancel_all_goals()
        break
