import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import line_helper as lh
import glob

def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image
    

def window_mask(width, height, img_ref, center,level):
    """
    Applies a window mask.
=
    """
    output = np.zeros_like(img_ref)
    output[int(img_ref.shape[0]-(level+1)*height):int(img_ref.shape[0]-level*height),max(0,int(center-width/2)):min(int(center+width/2),img_ref.shape[1])] = 1
    return output
    
def find_lines_windows(binary_warped, draw_boxes=True):
    """
    Find line lanes
    Applies a histogram to the image, and splits for left and right
    
    
    Source: Udacity
    """
	# Assuming you have created a warped binary image called "binary_warped"
	# Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]*.45:,:], axis=0)
	# Create an output image to draw on and  visualize the result
    out_img = np.uint8(np.dstack((binary_warped, binary_warped, binary_warped))*255)
	# Find the peak of the left and right halves of the histogram
	# These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

	# Choose the number of sliding windows
    nwindows = 9
	# Set height of windows
    window_height = np.int(binary_warped.shape[0]/nwindows)
	# Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
	# Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
	# Set the width of the windows +/- margin
    margin = 60
	# Set minimum number of pixels found to recenter window
    minpix = 50
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

	# Step through the windows one by one
    for window in range(nwindows):
	    # Identify window boundaries in x and y (and right and left)
	    win_y_low = binary_warped.shape[0] - (window+1)*window_height
	    win_y_high = binary_warped.shape[0] - window*window_height
	    win_xleft_low = leftx_current - margin
	    win_xleft_high = leftx_current + margin
	    win_xright_low = rightx_current - margin
	    win_xright_high = rightx_current + margin
	    # Draw the windows on the visualization image
	    if draw_boxes:
		    cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
		    cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
	    # Identify the nonzero pixels in x and y within the window
	    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
	    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
	    # Append these indices to the lists
	    left_lane_inds.append(good_left_inds)
	    right_lane_inds.append(good_right_inds)
	    # If you found > minpix pixels, recenter next window on their mean position
	    if len(good_left_inds) > minpix:
	        leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
	    if len(good_right_inds) > minpix:        
	        rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

	# Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

	# Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

	# Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
	# Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
 
	# Color the left lane red and the right lane blue
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = (255, 0, 0)
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = (0, 0, 255)
	# Return the result image
    return out_img, left_fitx, right_fitx, left_fit, right_fit, ploty, histogram
    
   
def find_curvature(image, leftx, rightx, left_fit, right_fit, ploty ):
    """
    FInd the curvature of 2 lines 
    
    Radius of curve = (1 + (2Ay + B)^2)^1.5 / |2A|
    source: Udacity & Slack
    """
    ploty = np.array(ploty).flatten()
    leftx = np.array(leftx).flatten()
    rightx = np.array(rightx).flatten()
    
    
    y_eval = np.max(ploty)
    left_curverad = ((1 + (2*left_fit[0]*y_eval + left_fit[1])**2)**1.5) / np.absolute(2*left_fit[0])
    right_curverad = ((1 + (2*right_fit[0]*y_eval + right_fit[1])**2)**1.5) / np.absolute(2*right_fit[0])

    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/700 # meters per pixel in x dimension

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])

    #print on image
    image = cv2.putText(image,'Curvature: ' + str(int(left_curverad)) + 'm', (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2,(255,0,0),4,cv2.LINE_AA)
    #image = cv2.putText(image,str(int(right_curverad)), (1000,100), cv2.FONT_HERSHEY_SIMPLEX, 2,(255,0,0),4,cv2.LINE_AA)

    # meters from center
    xm_per_pix = 3.7/700 # meteres per pixel in x dimension
    screen_middel_pixel = image.shape[1]/2
    left_lane_pixel = rightx[719]    # x position for left lane
    right_lane_pixel = leftx[719]   # x position for right lane
    car_middle_pixel = int((right_lane_pixel + left_lane_pixel)/2)
    screen_off_center = screen_middel_pixel-car_middle_pixel
    meters_off_center = xm_per_pix * screen_off_center
    image = cv2.putText(image,str('Distance from Center: ' +'%.2f' % meters_off_center + 'm'), (100,200), cv2.FONT_HERSHEY_SIMPLEX, 2,(255,0,0),4,cv2.LINE_AA)
    return image

def abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(0, 255)):
    """
    Sobel Absolute Thresold
    Calculate sobel for x and y (only return x or y)
    Calculate absolte value and apply threshold
    
    Source: Udacity
    """
    # 1) Take the derivative in x or y given orient = 'x' or 'y'
    sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0)
    sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1)
    # 2) Take the absolute value of the derivative or gradient
    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    # 3) Scale to 8-bit (0 - 255) then convert to type = np.uint8
    scaled_sobel_x = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    scaled_sobel_y = np.uint8(255*abs_sobely/np.max(abs_sobely))
    # 4) Create a mask of 1's where the scaled gradient magnitude 
            # is > thresh_min and < thresh_max
    thresh_min = thresh[0]
    thresh_max = thresh[1]
    if orient == 'x':
        binary_output = np.zeros_like(scaled_sobel_x)
        binary_output[(scaled_sobel_x >= thresh_min) & (scaled_sobel_x <= thresh_max)] = 1
    elif orient == 'y':
        binary_output = np.zeros_like(scaled_sobel_y)
        binary_output[(scaled_sobel_y >= thresh_min) & (scaled_sobel_y <= thresh_max)] = 1
        
    # 6) Return this mask as your binary_output image
    return binary_output

def mag_thresh(gray, sobel_kernel=3, mag_thresh=(0, 255)):
    """
    Sobel Magniture Threshold
    Calulate Sobel x and y, calculate the magnitude and then apply a threshold
    
    Source: Udacity
    
    """
    # Take both Sobel x and y gradients
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Calculate the gradient magnitude
    gradient_mag = np.sqrt(sobelx**2 + sobely**2)
    # Rescale to 8 bit
    gradient_mag = (gradient_mag/(np.max(gradient_mag)/255)).astype(np.uint8) 
    # Create a binary image of ones where threshold is met, zeros otherwise
    binary_output = np.zeros_like(gradient_mag)
    binary_output[(gradient_mag >= mag_thresh[0]) & (gradient_mag <= mag_thresh[1])] = 1

    return binary_output

def dir_threshold(gray, sobel_kernel=3, thresh=(0, np.pi/2)):
    """
    Sobel Direction Thresold
    Calculate Sobel x and y
    Calculate the gradient and then apply a threshold
    
    Source: Udacity
    
    """
    # 1) Take the gradient in x and y separately
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # 2) Take the absolute value of the x and y gradients
    abs_sobelx = abs(sobelx)
    abs_sobely = abs(sobely)
    # 3) Use np.arctan2(abs_sobely, abs_sobelx) to calculate the direction of the gradient 
    gradient = np.arctan2(abs_sobely, abs_sobelx)
    # 4) Create a binary mask where direction thresholds are met
    binary_output = np.zeros_like(gradient)
    binary_output[(gradient >= thresh[0]) & (gradient <= thresh[1])] = 1
    # 6) Return this mask as your binary_output image
    return binary_output

def get_camera_calibration():
    """
    Performs calibration of the camera 
    given a set of images (checkerboard)
    
    Source: Udacity
    
    """
    image_files = glob.glob('./camera_cal/calibration*.jpg')
    nx = 9
    ny = 6
    
    obj_points = []
    img_points = []
    objp = np.zeros((nx*ny,3), np.float32)
    objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2)
    shape = (0,0)
    
    for fname in image_files:
        img = mpimg.imread(fname)
    
        # Use cv2.calibrateCamera and cv2.undistort()
        gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (nx,ny),None)
        shape = (gray.shape[0], gray.shape[1])
        if ret == True:
            img_points.append(corners)
            obj_points.append(objp)
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, shape, None, None)
    return mtx, dist

def undistort(img, mtx, dist):
    """
    Undistorts image given camera calibration
    
    """
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    return undist

    
def draw_lines(image, undist, Minv, left_fitx, right_fitx, ploty):
    """
    Draws lines in image
    
    Source: Udacity
    
    """
    
    warped = np.copy(image)
    ploty = np.array(ploty).flatten()
    left_fitx = np.array(left_fitx).flatten()
    right_fitx = np.array(right_fitx).flatten()
    
    # Create an image to draw the lines on
    warp_zero = np.zeros_like(warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    #cv2.fillPoly(color_warp, np.int_([pts_left]), (255,0, 0))
    #cv2.fillPoly(color_warp, np.int_([pts_right]), (0,0, 255))
    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, Minv, (color_warp.shape[1], color_warp.shape[0])) 
    # Combine the result with the original image
    result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
    return result, color_warp
    
def hls_select(img, thresh=(0, 255)):
    """
    Converts image to HLS
    Thresholds the S-channel of HLS
    
    """
    #convert to HLS color space
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    
    S = hls[:,:,2]
    #not suring Lightness for now
    L = hls[:,:,1]
    
    binary = np.zeros_like(S)  
    #apply threshold
    binary[(S > thresh[0]) & (S <= thresh[1])] = 1
   
    return binary
  
    
def gaussian_blur(img, kernel_size=3):
    """
    Applies guassian kernel to image  
    """
    blur_image = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    return blur_image
    
def grayscale(img):
    """
    Convert to grayscale 
    """    
    gray_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    return gray_image
    
        
def get_projective_transform(src, dst):
    """
    Calculate the projective tranform and inverse
    
    Source: Udacity
    
    """
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    return M, Minv
    
def apply_projective_transform(M, image):
    """
    Apply projective tranform to image
    
    Source: Udacity
    
    """
    image_size = (image.shape[1], image.shape[0])
    #print(img_size)
    warped_image = cv2.warpPerspective(image, M, image_size, flags=cv2.INTER_NEAREST)
    return warped_image
    
def convert_to_3channel(image):
    """
    Covert single channel image to 3 channels
    
    """
    im = np.zeros((720,1280,3))
    
    im[:,:,0] = image/np.max(image) * 255
    im[:,:,1] = image/np.max(image) * 255
    im[:,:,2] = image/np.max(image) * 255

    return im
    
def get_yellow_and_white_image(image):
    
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    # define range of yellow and white color in HSV
    lower_yellow = np.array([20, 100, 100], np.uint8)
    upper_yellow = np.array([30, 255, 255], np.uint8)    
    lower_white = np.array([0, 0, 125], np.uint8)
    upper_white = np.array([0, 0, 255], np.uint8)
    
    # Threshold the HSV image to get only blue colors
    yellow_image = cv2.inRange(hsv, lower_yellow , upper_yellow )
    white_image = cv2.inRange(hls, lower_yellow , upper_yellow ) 
    white_image = hls_select(image, thresh=(125,255))
    yellow_and_white_image = cv2.bitwise_or(yellow_image,white_image)
    
    return yellow_and_white_image
      