
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
from moviepy.editor import VideoFileClip

#Global variables
mtx = []
dist = []
M = []
line_left = None
line_right = None

class Prev_Lines():
    """
    Class to keep last n Lines
    Circular List implementation
    
    """
    def __init__(self):
        self.line_list = []
        self.index = 0
        self.full_list = False
        self.count = 8
        
    def add_line(self, ln):
        """
        Adds a Line to the list
    
        """
        if self.full_list == False:
            self.line_list.append(ln)
            self.index = (self.index + 1) 
            if self.index >= self.count:
                self.full_list = True
                self.index = self.index  % self.count 
            
        else:
            self.line_list[self.index] = ln
            self.index = (self.index + 1) % self.count
        
            
    def get_line_data(self):
        """
        Calculates the Line data given all line data in list
    
        """
        new_line = Line()
        
        for ln in self.line_list :
            new_line.allx = new_line.allx +ln.recent_xfitted
            new_line.best_fit = new_line.best_fit + ln.current_fit
            new_line.ally = ln.ally
        if self.full_list == True:
            new_line.best_fit = new_line.best_fit / self.count
            new_line.allx = new_line.allx / self.count
        else:
            new_line.best_fit = new_line.best_fit / self.index
            new_line.allx = new_line.allx / self.index
            
        return new_line
        
               
# Define a class to receive the characteristics of each line detection
class Line():
    """
    Class to hold characteristics for line detection
    
    """
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False  
        # x values of the last n fits of the line
        self.recent_xfitted = [] 
        #average x values of the fitted line over the last n iterations
        self.bestx = None     
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = np.array([0,0,0], dtype='float') 
        #polynomial coefficients for the most recent fit
        self.current_fit = np.array([0,0,0], dtype='float') 
        #radius of curvature of the line in some units
        self.radius_of_curvature = None 
        #distance in meters of vehicle center from the line
        self.line_base_pos = None 
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float') 
        #x values for detected line pixels
        self.allx = np.zeros(720)
        #y values for detected line pixels
        self.ally = None

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
    histogram = np.sum(binary_warped[binary_warped.shape[0]/2:,:], axis=0)
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
    image = cv2.putText(image,'Curvature: ' + str(int(left_curverad)) + 'meters', (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2,(100,100,100),4,cv2.LINE_AA)
    #image = cv2.putText(image,str(int(right_curverad)), (1000,100), cv2.FONT_HERSHEY_SIMPLEX, 2,(255,0,0),4,cv2.LINE_AA)

    # meters from center
    xm_per_pix = 3.7/700 # meteres per pixel in x dimension
    screen_middel_pixel = img.shape[1]/2
    left_lane_pixel = rightx[719]    # x position for left lane
    right_lane_pixel = leftx[719]   # x position for right lane
    car_middle_pixel = int((right_lane_pixel + left_lane_pixel)/2)
    screen_off_center = screen_middel_pixel-car_middle_pixel
    meters_off_center = xm_per_pix * screen_off_center
    image = cv2.putText(image,str('Radius: ' +'%.5f' % meters_off_center + 'm'), (100,200), cv2.FONT_HERSHEY_SIMPLEX, 2,(100,100,100),4,cv2.LINE_AA)
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
    cv2.fillPoly(color_warp, np.int_([pts_left]), (255,0, 0))
    cv2.fillPoly(color_warp, np.int_([pts_right]), (0,0, 255))
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
    # 1) Convert to HLS color space
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    S = hls[:,:,2]
    L = hls[:,:,1]
    # 2) Apply a threshold to the S channel
    # 3) Return a binary image of threshold result
    binary = np.zeros_like(L)
    binary[(L > thresh[0]) & (L <= thresh[1])] = 1
    #binary_output = np.copy(img) # placeholder line
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
    i = np.zeros((720,1280,3))
    
    i[:,:,0] = image/np.max(image) * 255
    i[:,:,1] = image/np.max(image) * 255
    i[:,:,2] = image/np.max(image) * 255

    return i

def process_image(img):
    """
    Processes images individually to identify lane lines
    
    Source: Udacity & Slack
    
    """
    
    #undistort image using camera calibration
    undistort_image = undistort(img, mtx, dist)
    
    #get example of undistorted image
    
    
    #blur image and convert to hsl and grayscale (for processing)
    blur_image = gaussian_blur(undistort_image)
    
    hsl_image = hls_select(blur_image, thresh=(150, 255))
    gray_image = grayscale(blur_image)
    gray_image[(gray_image < 200)] = 0
    
    # Define kernel size
    kernel_size= 5
    #combine all sobel thresholding functions
    abs_sobel_thresh_x = abs_sobel_thresh(gray_image, orient='x', sobel_kernel=kernel_size, thresh=(10, 255))
    abs_sobel_thresh_y = abs_sobel_thresh(gray_image, orient='y', sobel_kernel=kernel_size, thresh=(40, 255))
    mag_sobel = mag_thresh(gray_image, sobel_kernel=kernel_size, mag_thresh=(15, 255))
    dir_sobel = dir_threshold(gray_image, sobel_kernel=kernel_size, thresh=(.45, 1.25))
    sobel_img = np.zeros_like(dir_sobel)
    sobel_img[((abs_sobel_thresh_x == 1) & (abs_sobel_thresh_y == 1)) | ((mag_sobel == 1) & (dir_sobel == 1))] = 1  
    
    #combine sobel and hsl images
    combine_image = np.zeros_like(sobel_img)
    combine_image[(hsl_image > 0) | (sobel_img > 0)] = 1
    
   
    
    #define region of interest
    xsize = hsl_image.shape[1]
    ysize = hsl_image.shape[0]
    vertices = np.array([[(0,ysize),(xsize*.4,ysize*.55), (xsize*.6,ysize*.55), (xsize,ysize)]], dtype=np.int32)
    #vertices = np.array([[(100,690),(600,300), (800,300), (1100,690)]], dtype=np.int32)
    roi_image = region_of_interest(combine_image, vertices)
    
    fig1 = plt.figure('Color and Region Processing')
    fig1.add_subplot(2,1,1)
    plt.imshow(combine_image)
    fig1.add_subplot(2,1,2)
    plt.imshow(roi_image)
    #plt.show()
    fig1.savefig('output_images/binary_combo_example.png')
    
    #apply projective transform
    birds_eye = apply_projective_transform(M, roi_image)
    
    b2 = np.copy(birds_eye)

    #find the lanes in the window
    out_image, left_fitx, right_fitx, left_fit, right_fit, ploty, histogram = find_lines_windows(birds_eye)
    cv2.imwrite('output_images/color_fit_lines.png', out_image)
    

    #calculate curvature and display on image
    line_image, color_warp = draw_lines(birds_eye, undistort_image, Minv, left_fitx, right_fitx, ploty)
    img_show = find_curvature(line_image, left_fitx, right_fitx, left_fit, right_fit, ploty)

    diagScreen = np.zeros((1080, 1920, 3), dtype=np.uint8)
    diagScreen[0:720, 0:1280] = img_show
    diagScreen[0:240, 1280:1600] = cv2.resize(convert_to_3channel(roi_image), (320,240), interpolation=cv2.INTER_AREA) 
    diagScreen[0:240, 1600:1920] = cv2.resize(convert_to_3channel(birds_eye), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[290:530, 1280:1600] = cv2.resize(convert_to_3channel(histogram), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[290:530, 1600:1920] = cv2.resize(color_warp, (320,240), interpolation=cv2.INTER_AREA)*4
    
    diagScreen[600:1080, 1280:1920] = cv2.resize(out_image, (640,480), interpolation=cv2.INTER_AREA)*4
    
    font = cv2.FONT_HERSHEY_COMPLEX
    middlepanel = np.zeros((120, 1280, 3), dtype=np.uint8)
    cv2.putText(middlepanel, 'HSL image.    |       sobel image       |       Combined image', (30, 60), font, 1, (255,0,0), 2)
    diagScreen[720:840, 0:1280] = middlepanel

    diagScreen[840:1080, 0:320] = cv2.resize(convert_to_3channel(hsl_image), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[840:1080, 370:690] = cv2.resize(convert_to_3channel(sobel_img), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[840:1080, 740:1060] = cv2.resize(convert_to_3channel(combine_image), (320,240), interpolation=cv2.INTER_AREA)

    

    return diagScreen
    #return img_show
    
def process_video(img):
    """
    Process video to identify driving lane in each frame
    
    Soure: Udacity & Slack
    
    """
    
    #undistort image using camera calibration
    undistort_image = undistort(img, mtx, dist)
    
    #blur image and convert to hsl and grayscale (for processing)
    blur_image = gaussian_blur(undistort_image)
    
    hsl_image = hls_select(blur_image, thresh=(150, 255))
    gray_image = grayscale(blur_image)
    gray_image[(gray_image < 200)] = 0
    
    # Define kernel size
    kernel_size= 5
    #combine all sobel thresholding functions
    abs_sobel_thresh_x = abs_sobel_thresh(gray_image, orient='x', sobel_kernel=kernel_size, thresh=(10, 255))
    abs_sobel_thresh_y = abs_sobel_thresh(gray_image, orient='y', sobel_kernel=kernel_size, thresh=(20, 255))
    mag_sobel = mag_thresh(gray_image, sobel_kernel=kernel_size, mag_thresh=(15, 255))
    dir_sobel = dir_threshold(gray_image, sobel_kernel=kernel_size, thresh=(.45, 1.25))
    sobel_img = np.zeros_like(dir_sobel)
    sobel_img[((abs_sobel_thresh_x == 1) & (abs_sobel_thresh_y == 1)) | ((mag_sobel == 1) & (dir_sobel == 1))] = 1  
    
    #combine sobel and hsl images
    combine_image = np.zeros_like(sobel_img)
    combine_image[(hsl_image > 0) | (sobel_img > 0)] = 1
    
    
    #define region of interest
    xsize = hsl_image.shape[1]
    ysize = hsl_image.shape[0]
    #vertices = np.array([[(0,ysize),(xsize*.4,ysize*.55), (xsize*.6,ysize*.55), (xsize,ysize),(xsize*.8,ysize),(xsize*.6,ysize*.8),(xsize*.4,ysize*.8),(xsize*.2,ysize)]], dtype=np.int32)
    vertices = np.array([[(100,690),(600,300), (800,300), (1100,690)]], dtype=np.int32)
    roi_image = region_of_interest(combine_image, vertices)
    
    #apply projective transform
    birds_eye = apply_projective_transform(M, np.copy(roi_image))
    
    b2 = np.copy(birds_eye)

    #find the lanes in the window
    out_image, left_fitx, right_fitx, left_fit, right_fit, ploty, histogram = find_lines_windows(b2)
    
    #get lines based on previous lines
    left = Line()
    left.current_fit = left_fit
    left.recent_xfitted = left_fitx
    left.ally = ploty
    line_left.add_line(left)
    
    right = Line()
    right.current_fit = right_fit
    right.recent_xfitted = right_fitx
    right.ally = ploty
    line_right.add_line(right)
    
    new_left = line_left.get_line_data()
    new_right = line_right.get_line_data()

    #calculate curvature and display on image
    
    
    img_show, color_warp = draw_lines(birds_eye, undistort_image, Minv, new_left.allx, new_right.allx, new_left.ally)
    curv_image = find_curvature(img_show, new_left.allx, new_right.allx, new_left.best_fit, new_right.best_fit, new_left.ally)

    diagScreen = np.zeros((1080, 1920, 3), dtype=np.uint8)
    diagScreen[0:720, 0:1280] = img_show
    diagScreen[0:240, 1280:1600] = cv2.resize(convert_to_3channel(roi_image), (320,240), interpolation=cv2.INTER_AREA) 
    diagScreen[0:240, 1600:1920] = cv2.resize(convert_to_3channel(birds_eye), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[290:530, 1280:1600] = cv2.resize(convert_to_3channel(histogram), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[290:530, 1600:1920] = cv2.resize(color_warp, (320,240), interpolation=cv2.INTER_AREA)*4
    
    diagScreen[600:1080, 1280:1920] = cv2.resize(out_image, (640,480), interpolation=cv2.INTER_AREA)*4
    
    font = cv2.FONT_HERSHEY_COMPLEX
    middlepanel = np.zeros((120, 1280, 3), dtype=np.uint8)
    cv2.putText(middlepanel, 'HSL image.    |       sobel image       |       Combined image', (30, 60), font, 1, (255,0,0), 2)
    diagScreen[720:840, 0:1280] = middlepanel

    diagScreen[840:1080, 0:320] = cv2.resize(convert_to_3channel(hsl_image), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[840:1080, 370:690] = cv2.resize(convert_to_3channel(sobel_img), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[840:1080, 740:1060] = cv2.resize(convert_to_3channel(combine_image), (320,240), interpolation=cv2.INTER_AREA)

    #return img_show
    return diagScreen

    
if __name__ == '__main__':
    
    #set global variables
    global line_left
    line_left = Prev_Lines()
    
    global line_right
    line_right = Prev_Lines()
    
    #load image to calculate prospective tranform
    img = mpimg.imread('./test_images/straight_lines1.jpg')
    
    #original image
    src = np.float32([[585,460],[200,720],[1125, 720],[695, 460]])

    #modified image
    dst = np.float32([[585,460],[200,720],[1125,720],[695,460]])
    
    img_size = (img.shape[1], img.shape[0])
    src = np.float32(
        [[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
        [((img_size[0] / 6) - 10), img_size[1]],
        [(img_size[0] * 5 / 6) + 60, img_size[1]],
        [(img_size[0] / 2 + 55), img_size[1] / 2 + 100]])
    dst = np.float32(
    	[[(img_size[0] / 4), 0],
    	[(img_size[0] / 4), img_size[1]],
    	[(img_size[0] * 3 / 4), img_size[1]],
    	[(img_size[0] * 3 / 4), 0]])

    global M
    M, Minv = get_projective_transform(src, dst)
    
    print('Source:')
    print(src)
    
    print('Destination:')
    print(src)
    
    draw_image = np.copy(img)
    fig1 = plt.figure('Projecive Transform')
    cv2.line(draw_image, (src[0,0], src[0,1]), (src[1,0], src[1,1]), (255,0,0))
    cv2.line(draw_image, (src[2,0], src[2,1]), (src[3,0], src[3,1]), (255,0,0))
    
    proj_image = apply_projective_transform(M, draw_image)
    
    fig1.add_subplot(2,1,1)
    plt.imshow(draw_image)
    fig1.add_subplot(2,1,2)
    plt.imshow(proj_image)
    #plt.show()
    fig1.savefig('output_images/warped_straight_lines.png')
    

    #camera calibration
    global mtx
    global dist
    mtx, dist = get_camera_calibration()
    #functionality for saving then loading later
    np.savetxt('mtx.txt', mtx)
    np.savetxt('dist.txt', dist)   
    #mtx = np.loadtxt("mtx.txt", delimiter=" ")   
    #dist = np.loadtxt("dist.txt", delimiter=" ")
    
    
    
    print('mtx:')
    print(mtx)
    
    print('dist:')
    print(dist)
    
    checkerboard = mpimg.imread('./camera_cal/calibration1.jpg')
    undistort_checkerboard = undistort(checkerboard, mtx, dist)
    undistort_image = undistort(img, mtx, dist)
    fig1 = plt.figure('Undistort Image')
    fig1.add_subplot(2,1,1)
    plt.imshow(checkerboard)
    fig1.add_subplot(2,1,2)
    plt.imshow(undistort_checkerboard)
    #plt.show()
    fig1.savefig('output_images/undistort_checkerboard_output.png')
    
    
    undistort_image = undistort(img, mtx, dist)
    fig1 = plt.figure('Undistort Image')
    fig1.add_subplot(2,1,1)
    plt.imshow(img)
    fig1.add_subplot(2,1,2)
    plt.imshow(undistort_image)
    #plt.show()
    fig1.savefig('output_images/undistort_output.png')
    


    image_files = glob.glob('./test_images/*.jpg')
    test_images = 1
    i = 0
    if test_images == 1:
        for fname in image_files:
            #
            img = mpimg.imread(fname)
            img_show = process_image(img)
            fig1 = plt.figure('Resulting images')
            plt.imshow(img_show)
            #plt.show()
            #cv2.imwrite('output_images/example_output' + str(i) +'.png', img_show)
            plt.savefig('output_images/example_output' + str(i) +'.png')
            i = i + 1


    white_output = 'output_images/project_video_processed.mp4'
    clip1 = VideoFileClip("project_video.mp4")
    clip2 = VideoFileClip("challenge_video.mp4")
    white_clip = clip1.fl_image(process_video)
    white_clip.write_videofile(white_output, audio=False)


    
    
    
    
    
    
    