# COLOUR DETECTION
1. localize the position of the glass in the image by using the track bar 
2. Detect the pouring cereal by colour decided by HSV values 
3. Ratio between the region on cereal and bounding box.
4. There is an angle between the camera and glass so the caclulation of percentage has a tolerance of about 5% based on experiments. Eg: if we want to pour 50% the result can be between 45 to 55. 

# Edge Detection

## Parameters used
- ```threshold``` - It is a variable used to store the gray scale pixel value for thresholding. All the pixels above this value is considered to be 255(white) and the all the pixels below this value is considered to be 0(black). Currently it is set to 20.
- ```scale_percent``` - It defines the size to which the image is to be resized for easier computation. Currently it is set to 10% which means the image would be scaled down to 10% of its original size.

## Operation
1. Intially, the bounding box is to be defined by selecting using mouse drag. Press "r" to redraw the bounding box and press "c" to confirm the bounding box and start the edge detection.
2. The initial frame is saved to be subtracted with eevry other frame. This is used for unwanted edges suppression. So once the bounding box is created, the glass should not be moved.
3. The input image is converted to gray scale and a 7x7 Gaussian blur is applied for noise suppression.
4. Then a sobel filter is applied in Y direction as we are interested in horizontal lines.
5. Then the image is thresholded using the ```threshold``` value and the white edges are thickened by applying a 7x7 Gaussian smoothing. Also the pixels corresponding to the edges of the glass are suppressed as it can be highly noisy.
6. Then the image is resized to a size defined by ```scale_percent``` and we are identifying the top two adjacent white lines(top edge) to calculate the level of content within the glass.
