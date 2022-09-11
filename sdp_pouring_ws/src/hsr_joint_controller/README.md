# Pouring action

## Parameters used
- ```self.level``` - It is the target level of content within the glass defined by the user.
- ```self.angle``` - It is a variable used to store the current wrist roll angle. It is passed into action client to control the joint state. Initially it is set to 0.02 which is its initial state.
- ```self.step``` - It is the angle by which the wrist roll angle increments. Whenever the force feedback detects sudden changes in weight, this value is reduced to 0.95 of its value. That is speed will reduce over time. Currently its value is set to 0.02
- ```max_rate_of_change``` - It is the threshold value for the difference between previous and current weight in grams. If the difference is greater than this value, then the intermediate stop and turn back by an angle of 10 times the self.step procedure is initiated. Currently the threshold is 10 grams.
