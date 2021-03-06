#!/usr/bin/env python

# The above line is essential for ros python file
# Use equation axx+bx+c
import sys, time
import numpy as np
import matplotlib.pyplot as plt

# import ros libraries
import rospy
import roslib
import Line

# import opencv
import cv2

# import type CompressedImage from sensor_msgs
from sensor_msgs.msg import CompressedImage

class Detected_Lane:
    leftLine = Line.Line()
    rightLine = Line.Line()
    space_accept = []

    # constructor
    def __init__(self):
        a = 1
        # self.subscriber = rospy.Subscriber("/Team1_image/compressed", CompressedImage, self.__callback__, queue_size = 1)
        
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

    
    def gaussian_blur(self, img, kernel_size=5):
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def binary_HSV(self, img):
        minThreshold = (0, 0, 180)
        maxThreshold = (179, 30, 255)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        out = cv2.inRange(hsv_img, minThreshold, maxThreshold)
        return out

    def shadow_HSV(self, img):
        minShadowTh = (90, 43, 36)
        maxShadowTh = (120, 81, 171)

        minLaneInShadow = (90, 43, 97)
        maxLaneInShadow = (120, 80, 171)

        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV);

        shadowMask = cv2.inRange(imgHSV, minShadowTh, maxShadowTh)

        shadow = cv2.bitwise_and(img,img, mask= shadowMask)

        shadowHSV = cv2.cvtColor(shadow, cv2.COLOR_BGR2HSV);

        out = cv2.inRange(shadowHSV, minLaneInShadow, maxLaneInShadow)
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

        height, width = img.shape
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

        min_x_left = width
        max_x_left = 0
        min_x_right = width
        max_x_right = 0
        min_x_mid = width
        max_x_mid = 0

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img.shape[0] - (window+1)*window_height
            win_y_high = img.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            if leftx_current < min_x_left:
                min_x_left = leftx_current
            win_xleft_high = leftx_current + margin
            if leftx_current > max_x_left:
                max_x_left = leftx_current
            win_xright_low = rightx_current - margin
            if rightx_current < min_x_right:
                min_x_right = rightx_current
            win_xright_high = rightx_current + margin
            if rightx_current > max_x_right:
                max_x_right = rightx_current
            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 1)
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 1)
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
        self.space_accept.append(min_x_left)
        self.space_accept.append(max_x_left)
        self.space_accept.append(min_x_right)
        self.space_accept.append(max_x_right)
        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        a = np.column_stack((leftx,lefty))
        b = np.column_stack((rightx,righty))
        # print(a)
        # print(b)
        # cv2.polylines(img, a, True, (255,0,0), 1)
        # cv2.polylines(img, b, True, (0,0,255), 1)
        # cv2.imshow('img', img)
        
        # Fit a second order polynomial to each
        try:
            left_fit = np.polyfit(leftx, lefty, 2)
        except np.RankWarning:
            left_fit = None

        try:
            right_fit = np.polyfit(rightx, righty, 2)
        except np.RankWarning:
            right_fit = None

        # print(left_fit)
        # print(right_fit)
        # Fit a second order polynomial to each
        # left_fit_m = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
        # right_fit_m = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
        left_fit_m = 0
        right_fit_m = 0

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        return left_fit, right_fit, left_fit_m, right_fit_m, out_img, a, b

    def __get_left_line__(self):
        best_fit_px, best_fit_m = self.leftLine.__get_line__()
        return best_fit_px
    
    def __get_right_line__(self):
        best_fit_px, best_fit_m = self.rightLine.__get_line__()
        return best_fit_px
    
    def __get_space_accept__(self):
        return self.space_accept

    def draw_arrpoint(self,img,arr,color=(0,0,255)):
        canvas = img.copy()
        for m in arr:
            y = m[1]
            x = m[0]
            canvas[y][x] = color
        return canvas

    def draw_quadratic(self,img,abc,color=(0,0,255)):
        canvas = img.copy()
        height,width = canvas.shape[:2]

        a,b,c = abc

        for x in range(width):
            y = int(a* x**2 + b* x + c)
            if y >= 0 and y < height:
                canvas[y][x] = color

        return canvas

    # callback function for processing image
    def __callback__(self, ros_data):
        # convert CompressedImage to int array
        np_arr = np.fromstring(ros_data.data, np.uint8)

        # decode image
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        SKY_LINE = 90
        CAR_LINE = 0
        HALF_ROAD = 30
        height,width = image_np.shape[:2]
        IMAGE_H = height - SKY_LINE - CAR_LINE
        IMAGE_W = width

        SRC_W1 = IMAGE_W/2 - HALF_ROAD/2
        SRC_W2 = IMAGE_W/2 + HALF_ROAD/2

        IMAGE_W1_TF = IMAGE_W/2 - HALF_ROAD
        IMAGE_W2_TF = IMAGE_W/2 + HALF_ROAD
        IMAGE_W3_TF = 0 - HALF_ROAD - 10
        IMAGE_W4_TF = IMAGE_W + HALF_ROAD + 10

        src = np.float32([[SRC_W1, 0], [SRC_W2, 0], [0, IMAGE_H], [width, IMAGE_H]])
        dst = np.float32([[IMAGE_W1_TF+12, 0], [IMAGE_W2_TF-12, 0], [IMAGE_W1_TF, height], [IMAGE_W2_TF, height]])
        image = image_np[SKY_LINE:(SKY_LINE+IMAGE_H-CAR_LINE), 0:IMAGE_W] # Apply np slicing for ROI crop

        warper_img = self.warper(image, src, dst)
        # cv2.imshow('warper_img', warper_img)

        binHSV = self.binary_HSV(warper_img)
        # cv2.imshow('binHSV', binHSV)

        shadow = self.shadow_HSV(warper_img)
        # cv2.imshow('shadow', shadow)

        result_img = binHSV + shadow
        # cv2.imshow('result_img', result_img)

        # combined_img = self.bin
        # test_img = cv2.bitwise_and(canny_img, combined_img)
        left_fit, right_fit, left_fit_m, right_fit_m, out_img,a,b = self.calc_line_fits(result_img)
        
        self.leftLine.__add_new_fit__(left_fit, left_fit_m)
        self.rightLine.__add_new_fit__(right_fit, right_fit_m)
        cv2.imshow('out_img', out_img)

        # d,e,f = left_fit
        # x,y,z = right_fit

        # m = 2*d*x
        # n = e*x+y*d
        # r = e*y/4+x*f/2+d*z/2-np.sqrt((e**2-4*d*f)*(y**2-4*x*z)/2)
        # mid = m,n,r
        canvas = np.zeros_like(out_img)
        canvas = self.draw_quadratic(canvas,left_fit)
        canvas = self.draw_quadratic(canvas,right_fit,color=(255,0,0))
        # canvas = self.draw_quadratic(canvas,mid,(0,255,0))
        cv2.imshow('canvas',canvas)

        arrr = np.zeros_like(out_img)
        arrr = self.draw_arrpoint(arrr,a)
        arrr = self.draw_arrpoint(arrr,b,color=(255,0,0))
        cv2.imshow('arrr',arrr)
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
