#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('agrobot')
import sys
import rospy
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


WIDTH = 800
LENGTH = 800

PROCESS_FRONT = True
PROCESS_DOWN = False
PROCESS_BACK = False

class AgrobotSensors:
    ''' 
    performs image processing on camera input from agrobot
    '''
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("agrobot/front_camera/image", Image, self.front_camera_callback)
        rospy.Subscriber("agrobot/downward_camera/image", Image, self.downward_camera_callback)

    def convert_cv_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        return cv_image
 
    def front_camera_callback(self, data):
        self.front_image = self.convert_cv_image(data)
        

    def downward_camera_callback(self, data):
        self.downward_image = self.convert_cv_image(data)
        self.process_sensor_data()
        
    def process_sensor_data(self):   
        cv2.imshow('front image', self.front_image)
        cv2.imshow('downward image', self.downward_image)
        cv2.waitKey(1)
 

def main(args):
  rospy.init_node('AgrobotSensors', anonymous=True)
  sensor = AgrobotSensors()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)