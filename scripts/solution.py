#!/usr/bin/env python

# The above line is essential for ros python file

import sys, time
import numpy as np
import matplotlib.pyplot as plt

# import ros libraries
import rospy
import roslib

# import opencv
import cv2

# import type CompressedImage from sensor_msgs
from sensor_msgs.msg import CompressedImage

class Detected_Lane:
    
    # constructor
    def __init__(self):
        
        self.subscriber = rospy.Subscriber("/Team1_image/compressed", CompressedImage, self.callback, queue_size = 1)

    def cannny(self, warped):
        # canny edges
        low_threshold = 20
        high_threshold = 150
        canny_edges = cv2.Canny(warped,low_threshold,high_threshold)
        return canny_edges

    def undistort(self, img, mtx, dist):
        undist = cv2.undistort(img, mtx, dist, None, mtx)
        return undist

    def camera_Calibration(self, img):
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.

        # Step through the list of images and search for chessboard corners in each one
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

        # If found, add object points, image points to the lists
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            
            # Draw and display the corners
            # img = cv2.drawChessboardCorners(img, (9,6), corners, ret)
        if objpoints and imgpoints: 
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            undist = self.undistort(img, mtx, dist)
            return undist
        return gray

    def add_points(img, src):
        img2 = np.copy(img)
        color = [255, 0, 0] # Red
        thickness = -1
        radius = 15
        x0, y0 = src[0]
        x1, y1 = src[1]
        x2, y2 = src[2]
        x3, y3 = src[3]
        cv2.circle(img2, (x0, y0), radius, color, thickness)
        cv2.circle(img2, (x1, y1), radius, color, thickness)
        cv2.circle(img2, (x2, y2), radius, color, thickness)
        cv2.circle(img2, (x3, y3), radius, color, thickness)
        return img2

    def add_lines(img, src):
        img2 = np.copy(img)
        color = [255, 0, 0] # Red
        thickness = 2
        x0, y0 = src[0]
        x1, y1 = src[1]
        x2, y2 = src[2]
        x3, y3 = src[3]
        cv2.line(img2, (x0, y0), (x1, y1), color, thickness)
        cv2.line(img2, (x1, y1), (x2, y2), color, thickness)
        cv2.line(img2, (x2, y2), (x3, y3), color, thickness)
        cv2.line(img2, (x3, y3), (x0, y0), color, thickness)
        return img2

    def warper(self, img, src, dst):        
        # Compute and apply perpective transform
        img_size = (img.shape[1], img.shape[0] + 90)
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, img_size)  # keep same size as input image
        return warped

    def unwarp(self, img, src, dst):       
        # Compute and apply inverse perpective transform
        img_size = (img.shape[1], img.shape[0])
        Minv = cv2.getPerspectiveTransform(dst, src)
        unwarped = cv2.warpPerspective(img, Minv, img_size)
        return unwarped

    # Define a function that applies Sobel x and y, 
    # then computes the magnitude of the gradient
    # and applies a threshold
    def calc_sobel(self, img, sx=False, sy=False, sobel_kernel=5, thresh=(25, 200)):
        
        # Convert to grayscale - sobel can only have one color channel
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        
        # Take the sobel gradient in x and y
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, sobel_kernel)
        
        if sx:
            abs_sobel = np.absolute(sobelx)
            scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
        elif sy:
            abs_sobel = np.absolute(sobely)
            scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
        else:
            # Calculate the magnitude 
            mag_sobel = np.sqrt(np.square(sobelx) + np.square(sobely))
        
            # Scale to 8-bit (0 - 255) and convert to type = np.uint8
            scaled_sobel = np.uint8(255*mag_sobel/np.max(mag_sobel))

        # Create a binary mask where mag thresholds are me
        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1   
        return scaled_sobel

    # Canny edge detector
    def canny(self, img, low_threshold, high_threshold):
        return cv2.Canny(img, low_threshold, high_threshold)

    def gaussian_blur(self, img, kernel_size=5):
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def run_canny(self, img, kernel_size=5, low_thresh=50, high_thresh=150):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        
        # Apply a Gaussian Blur
        gausImage = self.gaussian_blur(gray, kernel_size)

        # Run the canny edge detection
        cannyImage = self.canny(gausImage, low_thresh, high_thresh)

        return cannyImage

    # Apply threshold funciton to take a channel and a threshold and return the binary image
    def applyThreshold(self, channel, thresh):
        # Create an image of all zeros
        # binary_output = np.zeros_like(channel)
        # binary_output = np.copy(channel)
        # Apply a threshold to the channel with inclusive thresholds 
        # binary_output[(channel >= thresh[0]) & (channel <= thresh[1])] = 1
        retval, threshold = cv2.threshold(channel, thresh[0], thresh[1], cv2.THRESH_BINARY)
        return threshold

    # RGB R threshold
    def rgb_rthresh(self, img, thresh=(125, 255)):
        # Pull out the R channel - assuming that RGB was passed in
        channel = img[:,:,0]
        # Return the applied threshold binary image
        return self.applyThreshold(channel, thresh)

    # HLS S Threshold
    def hls_sthresh(self, img, thresh=(125, 255)):
        # Convert to HLS
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        # Pull out the S channel
        channel = hls[:,:,2]
        # Return the applied threshold binary image
        return self.applyThreshold(channel, thresh)

    # B-channel of LAB
    def lab_bthresh(self, img, thresh=(125, 255)):
        # Convert to HLS
        lab = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
        # Pull out the B channel
        channel = lab[:,:,2]
        # Return the applied threshold binary image
        return self.applyThreshold(channel, thresh)

    # L-channel of LUV
    def luv_lthresh(self, img, thresh=(125, 255)):
        # Convert to HLS
        luv = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
        # Pull out the L channel
        channel = luv[:,:,0]
        # Return the applied threshold binary image
        return self.applyThreshold(channel, thresh)

    def hsv_vthresh(self, img, thresh=(125, 255)):
        # Convert to HLS
        luv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # Pull out the V channel
        channel = luv[:,:,2]
        # Return the applied threshold binary image
        return self.applyThreshold(channel, thresh)

    def binaryPipeline(self, img, show_images=False, \
                   sobel_kernel_size=7, sobel_thresh_low=35, sobel_thresh_high=50, \
                   canny_kernel_size=5 , canny_thresh_low=50, canny_thresh_high=150, \
                   r_thresh_low=225, r_thresh_high=255, \
                   s_thresh_low=220, s_thresh_high=250, \
                   b_thresh_low=175, b_thresh_high=255, \
                   l_thresh_low=215, l_thresh_high=255 \
                  ):
        # Copy the image
        img2 = np.copy(img)

        # Undistort the image based on the camera calibration
        # undist = undistort(img, mtx, dist)

        # warp the image based on our perspective transform
        # warped = self.warper(img2)
        warped = img2
        
        ### COLOR SELECTION
        # Get the Red and saturation images
        r = self.rgb_rthresh(warped, thresh=(r_thresh_low, r_thresh_high))
        # cv2.imshow('r', r)
        s = self.hls_sthresh(warped, thresh=(s_thresh_low, s_thresh_high))
        # cv2.imshow('s', s)
        b = self.lab_bthresh(warped, thresh=(b_thresh_low, b_thresh_high))
        # cv2.imshow('b', b)
        l = self.luv_lthresh(warped, thresh=(l_thresh_low, l_thresh_high))
        # cv2.imshow('l', l)
        v = self.hsv_vthresh(warped)
        # cv2.imshow('v', v)

        ### EDGE DETECTION
        # Run the sobel magnitude calculation
        # edge = self.calc_sobel(warped, sx=True, sobel_kernel=sobel_kernel_size, thresh=(sobel_thresh_low, sobel_thresh_high))
        
        # Run canny edge detector
        edge = self.run_canny(warped, kernel_size=canny_kernel_size, low_thresh=canny_thresh_low, high_thresh=canny_thresh_high)

        # combine these layers
        combined_binary = np.zeros_like(r)
        # combined_binary[ (r == 1) | (s == 1) | (b == 1) | (l == 1) | (edge == 1) ] = 1
        # combined_binary[ (r == 1) | (s == 1) | (b == 1) | (l == 1) ] = 1
        combined_binary = r + s + b + l + edge
        return combined_binary

    def splitShadow(self, img):
        thresh=(40, 255)
        out = self.applyThreshold(img, thresh)
        return out

    def binary_HSV(self, img):
        minThreshold = (0, 0, 180);
        maxThreshold = (179, 30, 255);
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        out = cv2.inRange(hsv_img, minThreshold, maxThreshold, cv2.THRESH_BINARY)
        return out

    def calc_line_fits(self, img):

        ym_per_pix = 3*8/720 # meters per pixel in y dimension, 8 lines (5 spaces, 3 lines) at 10 ft each = 3m
        xm_per_pix = 3.7/550 # meters per pixel in x dimension, lane width is 12 ft = 3.7 meters
        ### Settings
        # Choose the number of sliding windows
        nwindows = 9
        # Set the width of the windows +/- margin
        margin = 20
        # Set minimum number of pixels found to recenter window
        minpix = 10


        # Take a histogram of the bottom half of the image
        histogram = np.sum(img[img.shape[0]//2:,:], axis=0)

        #plt.figure()
        #plt.plot(histogram)
        
        # Create an output image to draw on and  visualize the result
        out_img = np.dstack((img, img, img))*255
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Set height of windows
        window_height = np.int(img.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img.shape[0] - (window+1)*window_height
            win_y_high = img.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
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
        
        # Fit a second order polynomial to each
        # left_fit_m = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
        # right_fit_m = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
        left_fit_m = 0
        right_fit_m = 0

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        return left_fit, right_fit, left_fit_m, right_fit_m, out_img

    # callback function for processing image
    def callback(self, ros_data):
        # convert CompressedImage to int array
        np_arr = np.fromstring(ros_data.data, np.uint8)

        # decode image
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        SKY_LINE = 90
        CAR_LINE = 30
        HALF_ROAD = 30
        height,width = image_np.shape[:2]
        IMAGE_H = height - SKY_LINE - CAR_LINE
        IMAGE_W = width

        SRC_W1 = IMAGE_W/2 - HALF_ROAD
        SRC_W2 = IMAGE_W/2 + HALF_ROAD

        IMAGE_W1_TF = IMAGE_W/2 - HALF_ROAD
        IMAGE_W2_TF = IMAGE_W/2 + HALF_ROAD
        IMAGE_W3_TF = 0 - HALF_ROAD - 10
        IMAGE_W4_TF = IMAGE_W + HALF_ROAD + 10

        src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        dst = np.float32([[IMAGE_W1_TF, height], [IMAGE_W2_TF, height], [IMAGE_W3_TF, 0], [IMAGE_W4_TF, 0]])
        image = image_np[SKY_LINE:(SKY_LINE+IMAGE_H-CAR_LINE), 0:IMAGE_W] # Apply np slicing for ROI crop

        warper_img = self.warper(image, src, dst)
        cv2.imshow('warper_img', warper_img)

        binHSV = self.binary_HSV(warper_img)
        cv2.imshow('binHSV', binHSV)

        sobel_img = self.calc_sobel(warper_img)
        cv2.imshow('sobel_img', sobel_img)

        splitShadow_img = self.splitShadow(sobel_img)
        cv2.imshow('splitShadow_img', splitShadow_img)

        canny_img = self.run_canny(warper_img)
        cv2.imshow('canny_img', canny_img)

        canny_splSd_img = np.bitwise_and(splitShadow_img, canny_img)
        canny_bin_img = np.bitwise_and(binHSV, canny_img)
        test1_img = canny_splSd_img + binHSV
        cv2.imshow('test_img', test1_img)
        cv2.imshow('canny_bin_img', canny_bin_img)

        # combined_img = self.binaryPipeline(warper_img, show_images=True)
        # cv2.imshow('combined_img', combined_img)

        # test_img = cv2.bitwise_and(canny_img, combined_img)
        left_fit, right_fit, left_fit_m, right_fit_m, out_img = self.calc_line_fits(test1_img)
        cv2.imshow('out_img', out_img)

        cv2.waitKey(2)

    

def main(args):
    ic = Detected_Lane()
    rospy.init_node('Detected_Lane', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Closing"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
