##Denise Miller
###Advanced Lane Finding Project - Self Driving Car - Udacity

---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/undistort_checkerboard_output.png "Undistorted"
[image2]: ./output_images/undistort_output.png "Road Transformed"
[image3]: ./output_images/binary_combo_example.png "Binary Example"
[image4]: ./output_images/warped_straight_lines.png "Warp Example"
[image5]: ./output_images/color_fit_lines.png "Fit Visual"
[image6]: ./output_images/example_output8.png "Output"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

###Camera Calibration

####1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

Camera Calibration is in lines 173-182 in LaneFinder.py and 247-277 in helper_functions.py._

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

camera calibration matrix:
[[  1.15158784e+03   0.00000000e+00   6.66166882e+02]
 [  0.00000000e+00   1.14506839e+03   3.86440329e+02]
 [  0.00000000e+00   0.00000000e+00   1.00000000e+00]]
distortion coefficientnts:
[ -2.35507028e-01  -7.90615255e-02  -1.28490540e-03   8.25840998e-05
   7.23166428e-02]

I then used the output `obj_points` and `img_points` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

###Pipeline (single images)

####1. Provide an example of a distortion-corrected image.
To correct for distortion, I applied the camera calibration correct to the image in LaneFinder.py line 28, and helper_functions lines 279-285._

![alt text][image2]
####2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.
The processing for thresholded binary image can be found in LaneFinder.py lines 31-53.
To create the thresholded binary image, I first applied a guassian blur, then converted the image to HLS (focus on saturation) and applied a threshold.  I also used all 3 Sobel functions, thresholding Sobel x and y, threhsolding the gradient, and thresholding the magnitude.  I then combined all them to identify the lane lines.  I then identified the region if interest.

![alt text][image3]

####3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for calculating the prospective tranform can be found in LaneFinder.py lines 127 - 134 and helper_functions.py lines 356-377.  This was done by using an image containing the lane lines, and identifying four points (2 lines) that should indicate parallel lines (square) in the "bird's eye" view.

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

Source:
[[  585.   460.]
 [  200.   720.]
 [ 1125.   720.]
 [  695.   460.]]
Destination:
[[ 320.    0.]
 [ 320.  720.]
 [ 960.  720.]
 [ 960.    0.]]

![alt text][image4]

####4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

To identify the lane line pixels I first took the histogram of the bottom half of the image, and split it in half (left side and right side).  I then found the peaks of the histogram, and used these as starting points for searching for lines.  I iterated through the height of the image, sliding a fixed size window to the x position where is is centered in the pixel blob.  The line made up by the center of these windows represents the line. This is found in lines helper_functions.py lines 33-127.

![alt text][image5]

####5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

The calculation of the curvature of the line was done using the lines identified in the previous section, and calcuating the curvature of each line using the equation: Radius of curve = (1 + (2Ay + B)^2)^1.5 / |2A|.  This was then converted from pixels to meters.  

The position with respect to the center of the lane was calculated as (image_center - center_of_lane_lines) * meters/pixel.

This can be found in lines helper_functions lines 130-170.

####6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

Drawing the lane area back to the image can be found in function draw_lines in lines 288-319 of helper_functions.py.  An example is below.  This also shows the diagnostic window.

![alt text][image6]

---

###Pipeline (video)

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./output_images/project_video_processed.mp4)


In order to determine where the pipeline has difficulties, I created a video with a diagnostic view.  
Here's a [link to my video diagnostic](./output_images/project_video_diagnostics.mp4)


---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

In order to improve the accuracy of the lines, I average the last 8 lines when calculating the line.  This can be found in line_helper.py.  The class Lines hold the characteristics for the lines.  The class Prev_Lines serves as a circular list for the line data, keeping track of the last 'count' lines.  This pipeline has some difficulty on roads that are very light or when there are large regions of missing lane lines.  It would also not perform well when changing lanes as the lanes may be out of the region of interest or outside of the image when applying the prospective transform._

