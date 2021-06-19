#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

import rospy
import sys
import numpy as np
from std_msgs.msg import Float64

class WheelController():

    def __init__(self):
        self.FL_position_pub = rospy.Publisher('/agrobot/wheel_FL_position_controller/command', Float64, queue_size=1)
        self.FL_velocity_pub = rospy.Publisher('/agrobot/wheel_FL_velocity_controller/command', Float64, queue_size=1)

    def publish_position(self, pos):
        position = Float64()
        position.data = pos
        print('position:', position)
        self.FL_position_pub.publish(position)

    def publish_velocity(self, vel):
        velocity = Float64()
        velocity.data = vel
        print('velocity:', velocity)
        self.FL_velocity_pub.publish(velocity)
    
def main(args):
    rospy.init_node('wheel steering', anonymous = True)
    rate = rospy.Rate(10)
    wc = WheelController()
    
    while not rospy.is_shutdown():
        try:
            print('enter pos:')
            pos = input()
            pos = float(pos)
            wc.publish_position(pos)
            print('enter velocity')
            vel = input()
            vel = float(vel)
            wc.publish_velocity(vel)
            rate.sleep()
        except:
            break
    

if __name__ == '__main__':
    print('started wheel steering')
    main(sys.argv)
