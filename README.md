<div align="center">
  <h1 align="center">Feedback for Pouring behaviour (Toyota HSR) </h1>
</div>
The aim of this project is to automate  day-to-day pouring task of cereals/ liquid. Robotic arms requires constant feedback through different sensors like vision, force for controlling pouring action.   
<div align="center"><img width="460" src="images/Pouring.png"></div>

<p align="center">
  You can find videos regarding the project <a href="https://drive.google.com/drive/folders/1kR5nTP0gXL9nWmM3-G-pLP0K_tlWbNDW">here</a>!
</p>

## Required Libraries:

1. ROS Noetic
2. Computer vision (OpenCV)
3. Python 3
4. Numpy

## ROS Architecture
![ROS_architecture](images/ROS_architecture.png)

## Execution steps

This repository has one workspace namely, sdp_pouring_ws. It consists of further three packages:

1. hsr_vision
2. hsr_force
3. hsr_joint_controller

Following sequence of commands to execute

<b>1.  </b>

```
source sdp_pouring_ws/devel/setup.bash
```

<b>2.) Feedback from force Sensor</b>

Run the force_sensor.py file to obtain the current weight in the grasped container. 

```
rosrun hsr_force force_sensor.py
```

<b>(i) Action performed:</b>

a. This command will bring the Lucy to its initial pouring position. Make sure Lucy is far away from table to prevent collision.

b. Lucy will ask to place the container between the gripper. Grasp at a position between mid to top region of the container. Selection of mid-to-top position for grasping in order to avoid collision with the cup or table. 

c. Weight of the grasped object would be announced as well as published (topic: \grams).

<b>3 Feedback from vision sensor</b>

<b>a.) Colour independent</b>

Run the edge detection file, if edge based detection algorithm (colour-independent) is to be performed)

```
rosrun hsr_vision edge_detection.py
```

<b>(i) Bounding box generation:</b>

a. This command will pop up the window of the current image. 

b. Draw bounding box around the cup by dragging the mouse around the cup (Eventhough the width of the glass is not entirely covered by the bounding box, make sure the entire height is covered for better accuracy). Press “r” to redraw the bounding box. Press “c” to confirm the bounding box and start publishing percentage.

<b>!!!</b> Make sure the glass position is not changed after the bounding box is set because the initial background image is subtracted for noise suppression.

<b>(ii) Following pop up windows appear:</b>

a. Cropped image of the selected region.

b. Sobel-y filter output subtracted with initial reference frame.

c. Thresholded and noise suppressed image.

d. A relative representation of the content within the glass.

<b>(iii) Action performed:</b>

a. Current liquid/cereal level in the cup is published (topic: \percent).

<b>b.) Colour dependent </b>

Run the colour detection file, if colour-mask based detection algorithm is to be performed. 

```
rosrun hsr_vision colour_detection.py
```
<b>(i) Bounding box generation:</b>

a. The above command will pop up the window of the current image as well as the track bar to adjust the bounding box.

b. Adjust the bounding box height and width from the track bar based on the cup selected. 

c. Once the bounding box is generated, it starts publishing percentage of filled cup. 

<b>(ii) Following pop up windows appear:</b>

a. Track bar for selecting HSV parameters to detect appropriate color.

b. Track bar for selecting height and width of the cup to detect level of the cereals/ liquid during pouring.

c. Real time view of the camera.

d. Mask would be displayed when lucy starts pouring liquid/cereal.

<b>(iii) Action performed:</b>

a. Current liquid/cereal level in the cup is published (topic: \percent).

<b>4.) Controlling pouring action based on the feedback received from different sensors.</b>

Run the pouring_action.py file along with the target level

```
rosrun hsr_joint_controller pouring_action.py --target user_input_level
```

This code controls the motion of wrist for pouring as well as desired level of liquid required in the cup. 

This subscribe to two topics:

a. \percent published by vision .

b. \grams published by force.

<b>(i) Action performed:</b>

a. The forward movement and the task completion condition is being handled by vision feedback.

b. The intermediate stop and go-back-a-bit action when weight changes beyond a level is being handled by force feedback.
