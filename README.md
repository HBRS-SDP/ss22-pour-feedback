# ss22-pour-feedback

Workspace name: sdp_pouring_ws. 

Three packages:

1. hsr_vision
2. hsr_force
3. hsr_joint_controller

# ROS Architecture
![ROS_architecture](images/ROS_architecture.png)

# Sequence to execute commands

<b>1. rosrun hsr_force force_sensor.py</b>

<b>Action performed:</b>

a. This command will bring the lucy to its initial pouring position.

b. Lucy will ask to place the object between the gripper. 

c. Weight of the grasped object would be announced as well as published (topic: \grams).

<b>2. rosrun hsr_vision bounding_box.py</b>

This will pop up the window of the current image. Draw bounding box around the cup by dragging the mouse around the cup. Press “r” to redraw the bounding box. Press “c” to confirm the bounding box and start publishing percentage.

<b>Action performed:</b>


<b>3. rosrun hsr_vision colour_detection.py</b>

Following pop up windows appear:

a. track bar for selecting HSV parameters to detect appropriate color.

b. Real time view of the camera.

c. Mask would be displayed when lucy starts pouring liquid/cereal

<b>Action performed:</b>

a. Current liquid/cereal level in the cup is published (topic: \percent)

<b>4. rosrun hsr_joint_controller pouring_action.py --target prefered_liquid_level</b>

This code controls the motion of wrist for pouring as well as desired level of liquid required in the cup. 

This subscribe to two topics:

a. \percent published by vision 

b. \grams published by force

<b>Action performed:</b>
