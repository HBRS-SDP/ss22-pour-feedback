# Pouring action

## Parameters used
- ```self.level``` - It is the target level of content within the glass defined by the user.
- ```self.angle``` - It is a variable used to store the current wrist roll angle. It is passed into action client to control the joint state. Initially it is set to 0.02 which is its initial state.
- ```self.step``` - It is the angle by which the wrist roll angle increments. Whenever the force feedback detects sudden changes in weight, this value is reduced to 0.95 of its value. That is speed will reduce over time. Currently its value is set to 0.02
- ```max_rate_of_change``` - It is the threshold value for the difference between previous and current weight in grams. If the difference is greater than this value, then the intermediate stop and turn back by an angle of 10 times the ```self.step``` procedure is initiated. Currently the threshold is 10 grams.

## Operation
1. The code subscribes to two values such as ```/percent``` and ```/grams```.
2. ```/percent``` publishes the current level of content within the glass and it is used to drive the wrist roll angle forward at a step size defined by ```self.step```. This is handled by the ```vision_callback()``` function.
3. ```/grams``` publishes the current weight in grams. It is compared with the previous weight value and checked whether it crosses the threshold defined by ```max_rate_of_change```. This is handled by the ```force_callback()``` function.
4. Whenever ```force_callback()``` function detects a bulk pouring, it will make wrist roll angle turn back by 10 times the current ```self.step``` value. Also the ```self.step``` is reduced to 0.95 of its value. The speed is reduced for the next forward motion which will be triggered by the ```vision_callback()```.
5. When the ```vision_callback()``` detects that the level is reached, it will notify the ```force_callback()``` to stop with the help of a flag. Also the robot wrist is moved back to its initial pouring position.
