#!/usr/bin/env python

# The above line is essential for ros python file

import sys, time
import numpy as np

# import ros libraries
import rospy
import roslib
import detectlane

# import opencv
import cv2

from std_msgs.msg import Float32

# import type CompressedImage from sensor_msgs
from sensor_msgs.msg import CompressedImage

class CarControl:
    carPos = (0., 0.)
    preError = 0.
    preVeloc = 30.
    preMid = 155
    laneWidth = 70
    steer_publisher = None
    speed_publisher = None

    # constructor
    def __init__(self):
        self.preMid = 155
        self.preError = 0.
        self.preVeloc = 30.
        self.laneWidth = 70
        self.preLeft = 120 
        self.preRight = 190
        self.carPos = (160., 208.)
        self.steer_publisher = rospy.Publisher('Team1_steerAngle', Float32, queue_size=10)
        self.speed_publisher = rospy.Publisher('Team1_speed', Float32, queue_size=10)

    def errorAngle(self, left = None, right = None, y = 175):
        mid = self.preMid
        if left is not None and right is not None:
            mid = (left + right)/2
        elif left is not None:
            mid = left + 35
        elif right is not None:
            mid = right - 35
        self.preMid = mid
        # print(left)
        # print(right)
        # print(mid)
        pi = np.arccos(-1.0)
        dx = mid - self.carPos[0]
        dy = self.carPos[1] - y
        if dx < 0:
            current_angle = -np.arctan(-dx / dy) * 180 / pi
        else: 
            current_angle = np.arctan(dx / dy) * 180 / pi
        # print(current_angle)
        angle = (self.preError + current_angle)/2
        return angle, 50
    

    def __driverCar__(self, left = [], right = [], check_point = [], velocity = 30.):
        # print(left)
        # print(right)
        error = self.preError
        velocity = self.preVeloc        
        if left != [] and right != [] and check_point != []:
            a,b = left[0]
            c,d = right[0]
            y = (check_point[0]+check_point[0])/2
            x_left = np.round((y-b)/a)
            x_right = np.round((y-d)/c)
            # print(x_left)
            # print(x_right)
            error, velocity = self.errorAngle(x_left, x_right, y)
        # print(error)
        # print(velocity)
        self.preError = error
        self.preVeloc = velocity
        self.steer_publisher.publish(error);
        self.speed_publisher.publish(velocity); 

def main(args):
    rospy.init_node('carcontrol', anonymous=True)
    rate = rospy.Rate(10)
    cc = CarControl()
    while not rospy.is_shutdown():
        cc.__driverCar__()
        rate.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass