#!/usr/bin/env python

# The above line is essential for ros python file

import sys, time
import numpy as np

# import ros libraries
import rospy
import roslib
import detectlane2
import carcontrol2

# import opencv
import cv2

from std_msgs.msg import Float32

# import type CompressedImage from sensor_msgs
from sensor_msgs.msg import CompressedImage   

detect = detectlane2.Detected_Lane()
car = carcontrol2.CarControl()

def imageCallback(data):
    try:
        cv2.waitKey(1)
        detect.__callback__(data)
        # print(detect.__get_left_line__())
        # print(detect.__get_right_line__())
        car.__driverCar__(detect.__get_left_line__(), detect.__get_right_line__(), detect.__get_check_point__())
    except Exception:
        print(Exception)

def main(args):
    
    subscriber = rospy.Subscriber("/Team1_image/compressed", CompressedImage, imageCallback, queue_size = 1)
    rospy.init_node('mynode', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Closing"
    cv2.destroyAllWindows()
        

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass