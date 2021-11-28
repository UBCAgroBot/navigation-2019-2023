#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

import rospy
import sys
import numpy as np
from std_msgs.msg import Float64, String
import numpy as np
import ast

from pid import PID

class WheelController():

    def __init__(self, P=0.0, I=0.0, D=0.0, current_time=None):
        self.FL_position_pub = rospy.Publisher('/agrobot/wheel_FL_position_controller/command', Float64, queue_size=1)
        self.FL_velocity_pub = rospy.Publisher('/agrobot/wheel_FL_velocity_controller/command', Float64, queue_size=1)
        self.FR_position_pub = rospy.Publisher('/agrobot/wheel_FR_position_controller/command', Float64, queue_size=1)
        self.FR_velocity_pub = rospy.Publisher('/agrobot/wheel_FR_velocity_controller/command', Float64, queue_size=1)
        self.BL_position_pub = rospy.Publisher('/agrobot/wheel_BL_position_controller/command', Float64, queue_size=1)
        self.BL_velocity_pub = rospy.Publisher('/agrobot/wheel_BL_velocity_controller/command', Float64, queue_size=1)
        self.BR_position_pub = rospy.Publisher('/agrobot/wheel_BR_position_controller/command', Float64, queue_size=1)
        self.BR_velocity_pub = rospy.Publisher('/agrobot/wheel_BR_velocity_controller/command', Float64, queue_size=1)
        self.sensor_sub = rospy.Subscriber('/agrobot/sensors_data', String, self.sensor_callback)
        self.theta_pid = PID(P=0.5, I=0.05, D=0.005)
        self.x_pid = PID(P=1.0, I=0.1, D=0.01)

    def sensor_callback(self, data):
        sensor_array = ast.literal_eval(data.data)
        
        fl_angle, fr_angle, bl_angle, br_angle  = self.calculate_angles(sensor_array)
        fl_velocity, fr_velocity, bl_velocity, br_velocity = self.calculate_speeds(sensor_array) 

        self.FL_position_pub.publish(Float64(fl_angle))
        self.FR_position_pub.publish(Float64(fr_angle))
        self.BL_position_pub.publish(Float64(bl_angle))
        self.BR_position_pub.publish(Float64(br_angle))

        self.FL_velocity_pub.publish(Float64(fl_velocity))
        self.FR_velocity_pub.publish(Float64(fr_velocity))
        self.BL_velocity_pub.publish(Float64(bl_velocity))
        self.BR_velocity_pub.publish(Float64(br_velocity))


    def calculate_angles(self, sensor_array):
        # centroid = sensor_array[0]
        
        # print(centroid)
        # alpha = 1.5
        # angle = alpha * (200 - centroid)/200 * np.pi/4
        # fl_angle, fr_angle, bl_angle, br_angle = angle, angle, angle, angle

        dx, dtheta = sensor_array[0], sensor_array[1]

        x_val = self.x_pid.update(-dx)
        theta_val = self.theta_pid.update(-dtheta)

        # x_val, theta_val = self.x_pid.output, self.theta_pid.output

        #turning
        beta = 3
        angle = beta* theta_val*np.pi/180 
        fl_angle, fr_angle = angle, angle
        bl_angle, br_angle = 0, 0

        
        alpha = 2
        shift =  alpha*x_val/100 * np.pi/4

        fl_angle += shift
        fr_angle += shift
        bl_angle += shift
        br_angle += shift

        print('dx', dx)
        print('dtheta', dtheta)
        print('x_val', x_val)
        print('theta_val', theta_val)
        print('shift:', shift)
        print('angle:', angle)
        print('fl angle, fr_angle:', fl_angle, fr_angle)
        print('bl angle, br_angle:', bl_angle, br_angle)
        print('\n\n')
        # shift = - dx/200 * np.pi/4
        # fl_angle = np.pi/4+np.pi/2
        # bl_angle = fl_angle + np.pi/2
        # br_angle = bl_angle + np.pi/2
        # fr_angle = br_angle + np.pi/2
        return fl_angle, fr_angle, bl_angle, br_angle
    
    def calculate_speeds(self, sensor_array):

        nominal_speed = 2.0
        return nominal_speed, nominal_speed, nominal_speed, nominal_speed




def main(args):
    rospy.init_node('wheel controller', anonymous = True)
    rate = rospy.Rate(10)
    wc = WheelController()
    
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except:
            break
    

if __name__ == '__main__':
    print('started wheel steering')
    main(sys.argv)
