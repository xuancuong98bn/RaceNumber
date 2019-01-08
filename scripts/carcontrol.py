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
        self.carPos = (160., 239.)
        self.steer_publisher = rospy.Publisher('Team1_steerAngle', Float32, queue_size=10)
        self.speed_publisher = rospy.Publisher('Team1_speed', Float32, queue_size=10)

    def errorAngle(self):
        mid = self.preMid
        pi = np.arccos(-1.0)
        dx = mid - self.carPos[0]
        dy = self.carPos[1] - 200
        if dx < 0:
            current_angle = -np.arctan(-dx / dy) * 180 / pi
        elif dx > 0: 
            current_angle = np.arctan(dx / dy) * 180 / pi
        angle = (self.preError + current_angle)/2
        return angle, 30
    

    def __driverCar__(self, left, right, space_accept = [], velocity = 30.):
        print("==")
        print(left)
        print(right)
        error = self.preError
        velocity = self.preVeloc
        try:     
            #     a,b,c = left
            #     x,y,z = right
            #     m = 2*a*x
            #     n = b*x+y*a
            #     r = b*y/4+x*c/2+a*z/2-np.sqrt((b**2-4*a*c)*(y**2-4*x*z)/2)
            if left is not None:
                a,b,c = left
                cleft = c-200
                delta_left = b**2 - 4*a*cleft
                x1_left = np.round((-b+np.sqrt(delta_left))/(2*a))
                x2_left = np.round((-b-np.sqrt(delta_left))/(2*a))
                self.preLeft = x1_left if x1_left > space_accept[0] and x1_left < space_accept[1] else x2_left

            if right is not None:
                a,b,c = right
                cright = c-200
                delta_right = b**2 - 4*a*cright
                x1_right = np.round((-b+np.sqrt(delta_right))/(2*a))
                x2_right = np.round((-b-np.sqrt(delta_right))/(2*a))
                self.preRight = x1_right if x1_right > space_accept[2] and x1_right < space_accept[3] else x2_right
    
            print(self.preLeft)
            print(self.preRight)
            self.preMid = (self.preLeft+self.preRight)/2
            print(self.preMid)
            error, velocity = self.errorAngle()
            print(error)
            self.preError = error
            self.preVeloc = velocity
        except Exception:
            print("huhu")
        self.steer_publisher.publish(self.preError);
        self.speed_publisher.publish(self.preVeloc); 

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