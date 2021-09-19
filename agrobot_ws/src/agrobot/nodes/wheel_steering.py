#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

import rospy
import sys
import numpy as np
from std_msgs.msg import Float64
import numpy as np
class WheelController():

    def __init__(self):
        self.FL_position_pub = rospy.Publisher('/agrobot/wheel_FL_position_controller/command', Float64, queue_size=1)
        self.FL_velocity_pub = rospy.Publisher('/agrobot/wheel_FL_velocity_controller/command', Float64, queue_size=1)
        self.FR_position_pub = rospy.Publisher('/agrobot/wheel_FR_position_controller/command', Float64, queue_size=1)
        self.FR_velocity_pub = rospy.Publisher('/agrobot/wheel_FR_velocity_controller/command', Float64, queue_size=1)
        self.BL_position_pub = rospy.Publisher('/agrobot/wheel_BL_position_controller/command', Float64, queue_size=1)
        self.BL_velocity_pub = rospy.Publisher('/agrobot/wheel_BL_velocity_controller/command', Float64, queue_size=1)
        self.BR_position_pub = rospy.Publisher('/agrobot/wheel_BR_position_controller/command', Float64, queue_size=1)
        self.BR_velocity_pub = rospy.Publisher('/agrobot/wheel_BR_velocity_controller/command', Float64, queue_size=1)

    def turn_left(self, velocity):
        angle = Float64()
        angle.data = np.pi/4
        angle_neg = Float64()
        angle_neg.data = -np.pi/4

        vel = Float64()
        vel.data = -velocity
        vel_neg = Float64()
        vel_neg.data = velocity
        self.FL_position_pub.publish(angle_neg)
        self.FR_position_pub.publish(angle)
        self.BL_position_pub.publish(angle)
        self.BR_position_pub.publish(angle_neg)

        self.FL_velocity_pub.publish(vel)
        self.FR_velocity_pub.publish(vel_neg)
        self.BL_velocity_pub.publish(vel)
        self.BR_velocity_pub.publish(vel_neg)

    def turn_right(self, velocity):
        self.turn_left(-velocity)

    def stop(self):
        vel = Float64()
        vel.data = 0.0
        self.FL_velocity_pub.publish(vel)
        self.FR_velocity_pub.publish(vel)
        self.BL_velocity_pub.publish(vel)
        self.BR_velocity_pub.publish(vel)

    def go_forward(self, velocity):
        angle = Float64()
        angle.data=0.0
        vel = Float64()
        vel.data = velocity
        self.FL_position_pub.publish(angle)
        self.FR_position_pub.publish(angle)
        self.BL_position_pub.publish(angle)
        self.BR_position_pub.publish(angle)

        self.FL_velocity_pub.publish(vel)
        self.FR_velocity_pub.publish(vel)
        self.BL_velocity_pub.publish(vel)
        self.BR_velocity_pub.publish(vel)
    
    def go_backward(self, velocity):
        self.go_forward(-velocity)

    def publish_position(self, pos, locations):
        position = Float64()
        position.data = pos/180*np.pi
        if 'FL' in locations:
            self.FL_position_pub.publish(position)
        if 'FR' in locations:
            self.FR_position_pub.publish(position)
        if 'BL' in locations:
            self.BL_position_pub.publish(position)
        if 'BR' in locations:
            self.BR_position_pub.publish(position)

    def publish_velocity(self, vel, locations):
        velocity = Float64()
        velocity.data = vel

        if 'FL' in locations:
            self.FL_velocity_pub.publish(velocity)
        if 'FR' in locations:
            self.FR_velocity_pub.publish(velocity)
        if 'BL' in locations:
            self.BL_velocity_pub.publish(velocity)
        if 'BR' in locations:
            self.BR_velocity_pub.publish(velocity)

def control_individual_wheels(wc):
    print('FL angle in degrees:')
    FL_angle = float(input())
    wc.publish_position(FL_angle, {'FL'})

    print('FR angle in degrees:')
    FR_angle = float(input())
    wc.publish_position(FR_angle, {'FR'})

    print('BL angle in degrees:')
    BL_angle = float(input())
    wc.publish_position(BL_angle, {'BL'})

    print('BR angle in degrees:')
    BR_angle = float(input())
    wc.publish_position(BR_angle, {'BR'})

    print('FL velocity:')
    FL_vel = float(input())
    

    print('FR velocity:')
    FR_vel = float(input())
    print('BL velocity:')
    BL_vel = float(input())
    print('BR velocity:')
    BR_vel = float(input())

    
    wc.publish_velocity(FL_vel, {'FL'})
    wc.publish_velocity(FR_vel, {'FR'})
    wc.publish_velocity(BL_vel, {'BL'})
    wc.publish_velocity(BR_vel, {'BR'})

def discrete_control(wc):
    print('command:')
    command = input()
    vel = 1.0
    if command == 8:
        print('go forward')
        wc.go_forward(vel*5)
    elif command == 2:
        wc.go_backward(vel)
    elif command == 4:
        wc.turn_left(vel*8)
    elif command == 6:
        wc.turn_right(vel*8)
    elif command == 5:
        wc.stop()

def main(args):
    rospy.init_node('wheel steering', anonymous = True)
    rate = rospy.Rate(10)
    wc = WheelController()
    
    while not rospy.is_shutdown():
        try:
            # control_individual_wheels(wc)
            discrete_control(wc)
            rate.sleep()
        except:
            break
    

if __name__ == '__main__':
    print('started wheel steering')
    main(sys.argv)
