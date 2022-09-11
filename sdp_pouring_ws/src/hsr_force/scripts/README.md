# USAGE
1. The force code is the first file that needs to be run.
2. The robot is now present in its initial position. 
3. The force code is used to  provide feedback to the joint controller for pouring action. 

# WORKING
1. The force code subscribes to the topic 'hsrb/wrist_wrench/compensated'.
2. It is to be noted that here we use compensated data as it accounts for self weight and is more accurate as compared to raw data.
3. The topic 'hsrb/wrist_wrench/compensated' provides force in three components:
- X
- Y
- Z
4. It is to be noted that 