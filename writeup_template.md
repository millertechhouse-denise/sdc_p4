##Writeup Template
###You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

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

[image1]: ./examples/undistort_output.png "Undistorted"
[image2]: ./test_images/test1.jpg "Road Transformed"
[image3]: ./binary_combo_example.jpg "Binary Example"
[image4]: ./warped_straight_lines.jpg "Warp Example"
[image5]: ./color_fit_lines.jpg "Fit Visual"
[image6]: ./example_output.jpg "Output"
[video1]: ./project_video_processed.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!
###Camera Calibration

####1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

In file LaneFinder.py, Camera Calibratin is lines XXX through XXX. 

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

mtx:
[[  1.15158784e+03   0.00000000e+00   6.66166882e+02]
 [  0.00000000e+00   1.14506839e+03   3.86440329e+02]
 [  0.00000000e+00   0.00000000e+00   1.00000000e+00]]
dist:
[ -2.35507028e-01  -7.90615255e-02  -1.28490540e-03   8.25840998e-05
   7.23166428e-02]

I then used the output `obj_points` and `img_points` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

###Pipeline (single images)

####1. Provide an example of a distortion-corrected image.
To correct for distortion, I applied the camera calibration correct to the image in LaneFinder.py lines XX through XX.
![alt text][image2]
####2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.
The processing for thresholded binary image can be found in LaneFinder.py lines XX through XX.
To create the thresholded binary image, I first converted the image to HSL and applied a thresholded (function in lines XX through XX).  I also used all 3 Sobel functions (lines XX through XX), thresholding Sobel x and y, threhsolding the gradient, and thrwsholding the magnitude.  I then combined all them to identify the lane lines.  I then identified the region if interest (lines XX through XX)

![alt text][image3]

####3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for calculating tte prospective tranform can be found in LaneFinder.py lines XX through XX.  This was done by using an image containing the lane lines

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

Source:
[[  529.   495.]
 [  244.   685.]
 [  760.   495.]
 [ 1056.   685.]]
Destination:
[[  529.   495.]
 [  244.   685.]
 [  760.   495.]
 [ 1056.   685.]]

![alt text][image4]

####4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

To identify the lane line pixels I applied a histogram (lines XX through XX).  

![alt text][image5]

####5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in lines # through # in my code in `my_other_file.py`

####6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in lines # through # in my code in `yet_another_file.py` in the function `map_lane()`.  Here is an example of my result on a test image:

![alt text][image6]

---

###Pipeline (video)

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video.mp4)

---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

In order to improve the accuracy of the lines, I average the last 8 lines when calculating the line.  This can be found in LaneFinder.py lines XX through XX.  The class Lines hold the characteristics for the lines (lines XX though XX).  The class Prev_Lines serves as a circular list for the line data, keeping track of the last 8 lines.  This pipeline has some difficulty on roads that are very light or when there are large regions of missing lane lines.  It would also not perform well when changing lanes as the lanes may be out of the region of interest or outside of the image when applying the prospective transform._

