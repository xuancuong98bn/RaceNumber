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
    preError = 0
    laneWidth = 50

    # constructor
    def __init__(self):
        
        self.carPos = (120., 300.)
        self.steer_publisher = rospy.Publisher('Team1_steerAngle', Float32, queue_size=10)
        self.speed_publisher = rospy.Publisher('Team1_speed', Float32, queue_size=10)

    def errorAngle(self, dst):
        if (dst[0] == self.carPos[0]):
            return 0
        if (dst[1] == self.carPos[1]):
            return -90 if dst[0] < self.carPos[0] else 90
        pi = np.arccos(-1.0)
        dx = dst[0] - self.carPos[0]
        dy = self.carPos[1] - dst[1] 
        if (dx < 0):
            return - np.arctan(-dx / dy) * 180 / pi
        return np.arctan(dx / dy) * 180 / pi;
    

    def __driverCar__(self, left = [], right = [], velocity = 20):
        i = len(left) - 11
        error = self.preError
        if i > 0:
            while not left[i] and not right[i]:
                i = i -1
                if (i < 0):
                    return
            if (left[i] and right[i]):
                error = self.errorAngle((left[i] + right[i]) / 2);
            elif left[i]:
                error = self.errorAngle(left[i] + (self.laneWidth / 2, 0));
            else:
                error = self.errorAngle(right[i] - (self.laneWidth / 2, 0));

        angle = error;
        speed = velocity;

        self.steer_publisher.publish(0);
        self.speed_publisher.publish(10);    

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