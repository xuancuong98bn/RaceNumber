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

    def errorAngle(self, left = None, right = None):
        print(left)
        print(right)
        mid = self.preMid
        if left is not None and right is not None:
            mid = (left + right)/2
        elif left is not None:
            mid = left + 35
        elif right is not None:
            mid = right - 35
        self.preMid = mid
        pi = np.arccos(-1.0)
        dx = mid - self.carPos[0]
        dy = self.carPos[1] - 170
        if dx < 0:
            current_angle = -np.arctan(-dx / dy) * 180 / pi
        else: 
            current_angle = np.arctan(dx / dy) * 180 / pi
        angle = (2* self.preError + current_angle)/3
        return angle, 50*(1-np.abs(angle/90))
    

    def __driverCar__(self, left = [], right = [], velocity = 30.):
        # print(left)
        # print(right)
        error = self.preError
        velocity = self.preVeloc        
        if left is not None and right is not None:
            print("both")
            c1 = left[2]-170
            c2 = right[2]-170
            delta_left = left[1]*left[1] - 4*left[0]*c1
            delta_right = right[1]*right[1] - 4*right[0]*c2
            x1_left = np.round((-left[1]+np.sqrt(delta_left))/(2*left[0]))
            x2_left = np.round((-left[1]-np.sqrt(delta_left))/(2*left[0]))
            x1_right = np.round((-right[1]+np.sqrt(delta_right))/(2*right[0]))
            x2_right = np.round((-right[1]-np.sqrt(delta_right))/(2*right[0]))
            print(x1_left)
            print(x2_left)
            print(x1_right)
            print(x2_right)
            if x1_left > 0 and x1_right > 0 and x1_right > x1_left and x1_right - x1_left < 90:
                print("1")
                self.preLeft = x1_left
                self.preRight = x1_right
            elif x1_left > self.preLeft-10 and x2_right < self.preRight+10 and x2_right > x1_left and x2_right - x1_left < 90:
                print("2")
                self.preLeft = x1_left
                self.preRight = x2_right
            elif x2_left > self.preLeft-10 and x1_right < self.preRight+10 and x1_right > x2_left and x1_right - x2_left < 90:
                print("3")
                self.preLeft = x2_left
                self.preRight = x1_right
            elif x2_left > self.preLeft-10 and x2_right < self.preRight+10 and x2_right > x2_left and x2_right - x2_left < 90:
                print("4")
                self.preLeft = x2_left
                self.preRight = x2_right
            error, velocity = self.errorAngle(self.preLeft, self.preRight)
        elif left is not None:
            print("left")
            c1 = left[2]-170
            delta_left = left[1]*left[1] - 4*left[0]*c1
            x1_left = np.round((-left[1]+np.sqrt(delta_left))/(2*left[0]))
            x2_left = np.round((-left[1]-np.sqrt(delta_left))/(2*left[0]))
            if x1_left >=0:
                error, velocity = self.errorAngle(x1_left)
            else:   
                error, velocity = self.errorAngle(x2_left) 
        elif right is not None:
            print("right")
            c2 = left[2]-170
            delta_right = right[1]*right[1] - 4*right[0]*c2
            x1_right = np.round((-right[1]+np.sqrt(delta_right))/(2*right[0]))
            x2_right = np.round((-right[1]-np.sqrt(delta_right))/(2*right[0]))
            if x1_right >= 0:
                error, velocity = self.errorAngle(None, x1_right)
            else:
                error, velocity = self.errorAngle(None, x2_right)
        print(error)
        print(velocity)
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