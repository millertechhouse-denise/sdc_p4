
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
from moviepy.editor import VideoFileClip
import line_helper as lh
import helper_functions as hp

#Global variables
mtx = []
dist = []
M = []
line_left = None
line_right = None

video = False
    
def process_image(img):
    """
    Process video to identify driving lane in each frame
    
    Soure: Udacity & Slack
    
    """
    #undistort image using camera calibration
    undistort_image = hp.undistort(img, mtx, dist)
    
    #blur image and convert to hsl and grayscale (for processing)
    blur_image = hp.gaussian_blur(undistort_image)
    
    hsl_image = hp.hls_select(blur_image, thresh=(100, 225))
    #hsl_image = hp.get_yellow_and_white_image(blur_image)
    gray_image = hp.grayscale(blur_image)
    #gray_image[(gray_image < 150)] = 0
    # Define kernel size
    kernel_size= 7
    #combine all sobel thresholding functions
    abs_sobel_thresh_x = hp.abs_sobel_thresh(gray_image, orient='x', sobel_kernel=kernel_size, thresh=(10, 255))
    abs_sobel_thresh_y = hp.abs_sobel_thresh(gray_image, orient='y', sobel_kernel=kernel_size, thresh=(40, 255))
    mag_sobel = hp.mag_thresh(gray_image, sobel_kernel=kernel_size, mag_thresh=(15, 255))
    dir_sobel = hp.dir_threshold(gray_image, sobel_kernel=kernel_size, thresh=(.45, 1.25))

    sobel_img = np.zeros_like(dir_sobel)
    sobel_img[((abs_sobel_thresh_x == 1) & (abs_sobel_thresh_y == 1)) | ((mag_sobel == 1) & (dir_sobel == 1))] = 1 
    
    #combine sobel and hsl images
    combine_image = np.zeros_like(sobel_img)
    combine_image[(hsl_image > 0) | (sobel_img > 0)] = 1
    
    
    #define region of interest
    xsize = hsl_image.shape[1]
    ysize = hsl_image.shape[0]
    #vertices = np.array([[(0,ysize),(xsize*.4,ysize*.55), (xsize*.6,ysize*.55), (xsize,ysize),(xsize*.8,ysize),(xsize*.6,ysize*.8),(xsize*.4,ysize*.8),(xsize*.2,ysize)]], dtype=np.int32)
    vertices = np.array([[(100,720),(600,300), (800,300), (1100,720)]], dtype=np.int32)
    roi_image = hp.region_of_interest(combine_image, vertices)
    
    #apply projective transform
    birds_eye = hp.apply_projective_transform(M, np.copy(roi_image))
    
    #find the lanes in the window
    out_image, left_fitx, right_fitx, left_fit, right_fit, ploty, histogram = hp.find_lines_windows(birds_eye)
    
    if video == True:
        #get lines based on previous lines
        left = lh.Line()
        left.current_fit = left_fit
        left.recent_xfitted = left_fitx
        left.ally = ploty
        line_left.add_line(left)
    
        right = lh.Line()
        right.current_fit = right_fit
        right.recent_xfitted = right_fitx
        right.ally = ploty
        line_right.add_line(right)
    
        new_left = line_left.get_line_data()
        new_right = line_right.get_line_data()

        #calculate curvature and display on image
    
    
        img_show, color_warp = hp.draw_lines(birds_eye, undistort_image, Minv, new_left.allx, new_right.allx, new_left.ally)
        curv_image = hp.find_curvature(img_show, new_left.allx, new_right.allx, new_left.best_fit, new_right.best_fit, new_left.ally)
    else:
        #calculate curvature and display on image
        img_show, color_warp = hp.draw_lines(birds_eye, undistort_image, Minv, left_fitx, right_fitx, ploty)
        curv_image = hp.find_curvature(img_show, left_fitx, right_fitx, left_fit, right_fit, ploty)

    diagScreen = np.zeros((1080, 1920, 3), dtype=np.uint8)
    diagScreen[0:720, 0:1280] = img_show
    diagScreen[0:240, 1280:1600] = cv2.resize(img, (320,240), interpolation=cv2.INTER_AREA) 
    diagScreen[0:240, 1600:1920] = cv2.resize(hp.convert_to_3channel(birds_eye), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[290:530, 1280:1600] = cv2.resize(hp.convert_to_3channel(histogram), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[290:530, 1600:1920] = cv2.resize(color_warp, (320,240), interpolation=cv2.INTER_AREA)*4
    
    diagScreen[600:1080, 1280:1920] = cv2.resize(out_image, (640,480), interpolation=cv2.INTER_AREA)*4
    
    font = cv2.FONT_HERSHEY_COMPLEX
    middlepanel = np.zeros((120, 1280, 3), dtype=np.uint8)
    cv2.putText(middlepanel, 'HSL image.    |       sobel image       |       Combined image', (30, 60), font, 1, (255,0,0), 2)
    diagScreen[720:840, 0:1280] = middlepanel

    diagScreen[840:1080, 0:320] = cv2.resize(hp.convert_to_3channel(hsl_image), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[840:1080, 370:690] = cv2.resize(hp.convert_to_3channel(sobel_img), (320,240), interpolation=cv2.INTER_AREA)
    diagScreen[840:1080, 740:1060] = cv2.resize(hp.convert_to_3channel(combine_image), (320,240), interpolation=cv2.INTER_AREA)

    #return img_show
    return diagScreen

    
if __name__ == '__main__':
    
    #set global variables
    global line_left
    line_left = lh.Prev_Lines()
    
    global line_right
    line_right = lh.Prev_Lines()
    
    #load image to calculate prospective tranform
    img = mpimg.imread('./test_images/straight_lines1.jpg')
    
    #original image
    src = np.float32([[585,460],[200,720],[1125, 720],[695, 460]])

    #modified image
    dst = np.float32([[320,0],[320,720],[960,720],[960,0]])

    global M
    M, Minv = hp.get_projective_transform(src, dst)
    
    print('Source:')
    print(src)
    
    print('Destination:')
    print(dst)
    
    draw_image = np.copy(img)
    fig1 = plt.figure('Projecive Transform')
    cv2.line(draw_image, (src[0,0], src[0,1]), (src[1,0], src[1,1]), (255,0,0))
    cv2.line(draw_image, (src[2,0], src[2,1]), (src[3,0], src[3,1]), (255,0,0))
    
    proj_image = hp.apply_projective_transform(M, draw_image)
    
    fig1.add_subplot(2,1,1)
    plt.imshow(draw_image)
    fig1.add_subplot(2,1,2)
    plt.imshow(proj_image)
    #plt.show()
    fig1.savefig('output_images/warped_straight_lines.png')
    
    #camera calibration
    global mtx
    global dist
    mtx, dist = hp.get_camera_calibration()
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
    undistort_checkerboard = hp.undistort(checkerboard, mtx, dist)
    undistort_image = hp.undistort(img, mtx, dist)
    fig1 = plt.figure('Undistort Image')
    fig1.add_subplot(2,1,1)
    plt.imshow(checkerboard)
    fig1.add_subplot(2,1,2)
    plt.imshow(undistort_checkerboard)
    #plt.show()
    fig1.savefig('output_images/undistort_checkerboard_output.png')
    
    
    undistort_image = hp.undistort(img, mtx, dist)
    fig1 = plt.figure('Undistort Image')
    fig1.add_subplot(2,1,1)
    plt.imshow(img)
    fig1.add_subplot(2,1,2)
    plt.imshow(undistort_image)
    #plt.show()
    fig1.savefig('output_images/undistort_output.png')
    
    global video
    video = False
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

    video = True
    output = 'output_images/project_video_diagnostic.mp4'
    #output = 'output_images/project_video_processed.mp4'
    clip1 = VideoFileClip("project_video.mp4")
    clip2 = VideoFileClip("challenge_video.mp4")
    white_clip = clip1.fl_image(process_image)
    white_clip.write_videofile(output, audio=False)


    
    
    
    
    
    
    