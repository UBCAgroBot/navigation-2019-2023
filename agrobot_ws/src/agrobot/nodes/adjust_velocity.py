#!/usr/bin/env python
from __future__ import print_function
from __future__ import division
import cv2 
import argparse
import roslib
import rospy
import sys
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import time

# taken from https://github.com/ivmech/ivPID
class PID:

    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

def nothing(x):
    pass

class velocity_control:

    def __init__(self):
        self.init_time = time.time()
        self.centroid_location = 0
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.centroid_location_sub = rospy.Subscriber('/centroid_location', Float32, self.callback)
        self.pid_controller = PID(0,0,0,time.time())
        
    def get_velocity(self, centroid):

        # turning_factor = pid_control(centroid)
        turning_factor = 1
        driving_speed = cv2.getTrackbarPos('driving speed', "PID Controller") / 10.0
        turning_speed = cv2.getTrackbarPos('turning speed', "PID Controller") / 10.0
        
        self.pid_controller.setKp(cv2.getTrackbarPos('proportional', "PID Controller"))
        self.pid_controller.setKd(cv2.getTrackbarPos('derivative', "PID Controller"))
        self.pid_controller.setKi(cv2.getTrackbarPos('integral', "PID Controller"))
        
        error = centroid - 640
        self.pid_controller.update(error, time.time())

        scaling_factor = cv2.getTrackbarPos('scale factor', "PID Controller")
        pid_factor = self.pid_controller.output / scaling_factor
        # print("pid factor ", pid_factor)

        velocity = Twist()
        velocity.linear.x = driving_speed
        velocity.angular.z = turning_speed*pid_factor
        
        return velocity
    


    def callback(self, data):
        self.centroid_location = data.data
        cv2.imshow("PID Controller", np.zeros((1,400,3), np.uint8))
        cv2.waitKey(1)
        cv2.createTrackbar('driving speed','PID Controller',0,100,nothing)   
        cv2.createTrackbar('turning speed','PID Controller',0,100,nothing)
        cv2.createTrackbar('proportional','PID Controller',0,100,nothing)
        cv2.createTrackbar('derivative','PID Controller',0,100,nothing)
        cv2.createTrackbar('integral','PID Controller',0,100,nothing)
        cv2.createTrackbar('scale factor','PID Controller',1000,5000,nothing)
        
        if (time.time() - self.init_time) < 2.0:
            # initial values for PID control
            cv2.setTrackbarPos('driving speed', 'PID Controller', 1)
            cv2.setTrackbarPos('turning speed', 'PID Controller', 1)
            cv2.setTrackbarPos('proportional', 'PID Controller', 1)
            cv2.setTrackbarPos('derivative', 'PID Controller', 0)
            cv2.setTrackbarPos('integral', 'PID Controller', 0)
            cv2.setTrackbarPos('scale factor', 'PID Controller', 1000)

        velocity = self.get_velocity(self.centroid_location)
        # print("speed: " + str(velocity.linear.x) + "  turn: " + str(velocity.angular.z) + "\n")
        self.velocity_pub.publish(velocity)

    
def main(args):
    rospy.init_node('velocity_adjuster', anonymous = True)
    vc = velocity_control()
    
    try:
        rospy.spin()
        stop_velocity = Twist()
        stop_velocity.linear.x = 0
        stop_velocity.angular.z = 0
        rospy.Publisher('/cmd_vel', Twist, queue_size=1).publish(stop_velocity)
        print('published')

    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    print('started adjust_velocity')
    main(sys.argv)
