#!/usr/bin/env python

# The above line is essential for ros python file
# Use equation ax+b

import sys, time
import numpy as np
import matplotlib.pyplot as plt

# import ros libraries
import rospy
import roslib
import Line2

# import opencv
import cv2

# import type CompressedImage from sensor_msgs
from sensor_msgs.msg import CompressedImage

class Detected_Lane:
    leftLine = Line2.Line()
    rightLine = Line2.Line()
    check_point = []

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
        minThreshold = (0, 0, 180);
        maxThreshold = (179, 30, 255);
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
        nwindows = 1
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
        height = img.shape[0]
        window_height = np.int(img.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Create empty lists to receive left and right lane pixel indices
        left_lane_equation = []
        right_lane_equation = []
        check_point = [img.shape[0]]
        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img.shape[0] - (window+1)*window_height
            win_y_high = img.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            check_point.append(win_y_low)
            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 1)
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 1)
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Change to x, y index
            leftx = nonzerox[good_left_inds]
            lefty = nonzeroy[good_left_inds]
            rightx = nonzerox[good_right_inds]
            righty = nonzeroy[good_right_inds]
            # Create quation with each window
            leftx_fix, lefty_fix, left_fix = self.fix_line(leftx, lefty, height)
            rightx_fix, righty_fix, right_fix = self.fix_line(rightx, righty, height)

            left_equation = np.polyfit(leftx_fix, lefty_fix, 1)
            right_equation = np.polyfit(rightx_fix, righty_fix, 1)
            # Append these indices to the lists
            left_lane_equation.append(left_equation)
            right_lane_equation.append(right_equation)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        # left_lane_equation = np.concatenate(left_lane_equation)
        # right_lane_equation = np.concatenate(right_lane_equation)

        
        # Fit a second order polynomial to each
        # left_fit_m = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
        # right_fit_m = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
        left_fit_m = 0
        right_fit_m = 0

        return left_lane_equation, right_lane_equation, left_fit_m, right_fit_m, out_img, check_point

    def fix_line(seft, linex, liney, height):
        line_fix = []
        linex_fix = []
        liney_fix = []
        for i in range(height):
                temp_list = []
                check = False
                for j in range(len(liney)):
                    if i == liney[j]:
                        temp_list.append(linex[j])
                        check = True
                if check:
                    x = np.mean(temp_list)
                    line_fix.append([x,i])
                    linex_fix.append(x)
                    liney_fix.append(i)
        return linex_fix, liney_fix, line_fix

    def __get_left_line__(self):
        best_fit_equation, check_point = self.leftLine.__get_line__()
        return best_fit_equation
    
    def __get_right_line__(self):
        best_fit_equation, check_point = self.rightLine.__get_line__()
        return best_fit_equation
    
    def __get_check_point__(self):
        return self.check_point

    def draw_arrpoint(self,img,arr,color=(0,0,255)):
        canvas = img.copy()
        for m in arr:
            y = m[1]
            x = m[0]
            canvas[y][x] = color
        return canvas

    def draw_line(self,img,lane_equation,color=(0,0,255)):
        canvas = img.copy()
        height,width = canvas.shape[:2]
        for i in range(len(self.check_point)-1):
            a = lane_equation[i][0]
            b = lane_equation[i][1]
            x1 = int((self.check_point[i] - b) / a)
            x2 = int((self.check_point[i+1] - b) / a)
            cv2.line(canvas, (x1,self.check_point[i]), (x2,self.check_point[i+1]), color)
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
        left_lane_equation, right_lane_equation, left_fit_m, right_fit_m, out_img, check_point = self.calc_line_fits(result_img)
        self.leftLine.__add_new_fit__(left_lane_equation, left_fit_m, check_point)
        self.rightLine.__add_new_fit__(right_lane_equation, right_fit_m, check_point)
        cv2.imshow('out_img', out_img)
        self.check_point = check_point

        canvas = np.zeros_like(out_img)
        canvas = self.draw_line(canvas,left_lane_equation)
        canvas = self.draw_line(canvas,right_lane_equation,color=(255,0,0))
        cv2.imshow('canvas',canvas)

        # arrr = np.zeros_like(out_img)
        # arrr = self.draw_arrpoint(arrr,a)
        # arrr = self.draw_arrpoint(arrr,b,color=(255,0,0))
        # cv2.imshow('arrr',arrr)
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
