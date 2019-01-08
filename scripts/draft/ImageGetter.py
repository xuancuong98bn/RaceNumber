#!/usr/bin/env python

# The above line is essential for ros python file

import sys, time
import numpy as np

# import ros libraries
import rospy
import roslib

# import opencv
import cv2

# import type CompressedImage from sensor_msgs
from sensor_msgs.msg import CompressedImage

class ImageGetter:
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
        cv2.imshow('cv_img', image_np)

        # show gray image
        image_gray = cv2.cvtColor(image_np, cv2.COLOR_RGB2GRAY)
        cv2.imshow('cv_gray_img',image_gray)

        hsv = cv2.cvtColor(image_np, cv2.COLOR_RGB2HSV)

        lower_range = np.array([0,0,0])
        upper_range = np.array([0,0,255])

        threshold_image = cv2.inRange(hsv, lower_range, upper_range)

        cv2.imshow('threshold_image', threshold_image)

        res = cv2.bitwise_and(image_np, image_np, mask= threshold_image)
        cv2.imshow('res_image',res)

        # cv2.imwrite()

        cv2.waitKey(2)

def main(args):
    ic = ImageGetter()
    rospy.init_node('ImageGetter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Closing"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)