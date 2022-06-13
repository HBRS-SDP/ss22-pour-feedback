import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
from std_msgs.msg import Int32
import sys


x = 0.02


def callback(data):

    rospy.loginfo('in callback')

    if int(data.data)<22:
    
    	pass
    	
    else:
    	p.positions = [0.29,-0.42, 0.03,-1.07,0.02]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Duration(3)
        traj.points = [p]
        goal.trajectory = traj

        # send message to the action server
        cli.send_goal(goal)
        cli.wait_for_result()

        cli.cancel_all_goals()

        rospy.signal_shutdown("reached the target level")
        sys.exit(0)


if __name__ == '__main__':
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

    #print("after server")

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

    p.positions = [0.29,-0.42, 0.03,-1.07,2.5]
    p.velocities = [0, 0, 0, 0, 0]
    p.time_from_start = rospy.Duration(60)
    traj.points = [p]
    goal.trajectory = traj

     # send message to the action server
    cli.send_goal(goal)

    rospy.Subscriber('/percent', Int32, callback, queue_size=1)

    rospy.spin()

