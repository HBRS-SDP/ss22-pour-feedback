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
4. The rotation of the wrist takes place about the Z axis and hence transformation should be applied to all components with respect to that. 
5. The root of the squared difference is calculated between the previous iteration and new iteration of X,Y and Z components and then passed on as one value of force which is used for weight calculation. 
6. The weight is calculated in grams. 
7. The topic '/grams' is published at a rate of 10Hz. A faster rate can slow down the conttroller as there would not be much difference between each published value and would increase the number of iterations. 

# USAGE OF FILTERS
1. As the data is noisy, filtering of data is required before and after the calculation of weights. 
2. The two filters are:
    - Low pass filtering: used for the X,Y and Z components to get rid of the high frequency noise.
    - Median filtering: used for the weights before publishing them so that the data can have lesser sharp peaks and then passed on to the controller. 
3. The parameters of low pass filters such as sampling frequency, cut off frequency and order were selected by trial and error. 
4. A kernel size of 5 was selected for the median filter as we noticed that it took these many iterations would provide a noticable change.  

