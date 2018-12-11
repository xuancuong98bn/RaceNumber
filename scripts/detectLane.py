#!/usr/bin/env python

# The above line is essential for ros python file

import sys, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
#from ipywidgets import interact, interactive, fixed

# %matplotlib qt
# %matplotlib inline

# import ros libraries
import rospy
import roslib
import glob

# import opencv
import cv2

# import type CompressedImage from sensor_msgs
from sensor_msgs.msg import CompressedImage

class Detected_Lane:
    # constructor
    def __init__(self):
        
        self.subscriber = rospy.Subscriber("/Team1_image/compressed", CompressedImage, self.callback, queue_size = 1)

    # callback function for processing image
    def callback(self, ros_data):
        # convert CompressedImage to int array
        np_arr = np.fromstring(ros_data.data, np.uint8)

        # decode image
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # show the decoded image
        # cv2.imshow('cv_img', image_np)

        #############################################################

        # show gray image
        image_gray = cv2.cvtColor(image_np, cv2.COLOR_RGB2GRAY)
        # cv2.imshow('image_gray',image_gray)

        #  gray and gaussian edges
        img_hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100], dtype = "uint8")
        upper_yellow = np.array([30, 255, 255], dtype="uint8")

        mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(image_gray, 200, 255)
        mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
        mask_yw_image = cv2.bitwise_and(image_gray, mask_yw)

        kernel_size = 3
        gauss_gray = cv2.GaussianBlur(mask_yw_image, (kernel_size, kernel_size), 0)

        # show birdview
        SKY_LINE = 90
        HALF_ROAD = 20
        image = gauss_gray
        height,width = image.shape[:2]
        IMAGE_H = height-SKY_LINE
        IMAGE_W = width
        IMAGE_W1_TF = IMAGE_W/2 - HALF_ROAD
        IMAGE_W2_TF = IMAGE_W/2 + HALF_ROAD

        src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        dst = np.float32([[IMAGE_W1_TF, height], [IMAGE_W2_TF, height], [0, 0], [IMAGE_W, 0]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        polys = np.array([[0, IMAGE_H], [0, 0], [IMAGE_W, 0], [IMAGE_W, IMAGE_H], [0, IMAGE_H]])
        ROI_Full = cv2.fillPoly(image, polys, np.array([0,0,0]), lineType=8, shift=0)
        cv2.imshow('ROI_Full', ROI_Full)
        image = image[SKY_LINE:(SKY_LINE+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop
        cv2.imshow('ROI', image)
        warped_img = cv2.warpPerspective(image, M, (IMAGE_W, height)) # Image warping
        cv2.imshow('bird_view', warped_img)
        # plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB)) # Show results
        # plt.show()

        # show binary
        ret, image_binary = cv2.threshold(image_gray, 200, 255, cv2.THRESH_BINARY);
        # cv2.imshow('image_binary',image_binary)
        
        # canny edges
        low_threshold = 20
        high_threshold = 150
        canny_edges = cv2.Canny(warped_img,low_threshold,high_threshold)
        cv2.imshow("canny_edges", canny_edges)

        # lower_range = np.array([0,0,0])
        # upper_range = np.array([0,0,255])

        # threshold_image = cv2.inRange(img_hsv, lower_range, upper_range)

        # cv2.imshow('threshold_image', threshold_image)

        # res = cv2.bitwise_and(image_np, image_np, mask= threshold_image)
        # cv2.imshow('res_image',res)

        # cv2.imwrite()

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
