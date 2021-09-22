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
import numpy as np

import os
from algorithms import HoughAlgorithm, MiniContoursAlgorithm, ScanningAlgorithm, CenterRowAlgorithm
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
        try:
            front_image = self.front_image
            downward_image = self.downward_image   
        except:
            print('did not see front or downward image')
            return
        
        front_cx, front_cy = self.get_centroid(front_image)
        
        front_image = cv2.circle(front_image, (front_cx, front_cy), 10, (0,255,255), 1)
        cv2.imshow('front image', front_image)
        cv2.imshow('downward image', downward_image)
        cv2.waitKey(1)
    
    def get_centroid(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ### Filter image and allow only shades of green to pass
        mask = cv2.inRange(hsv, np.array([31, 43, 23]), np.array([255, 255, 100]))
        ### Apply gaussian blur (can be removed)
        mask = cv2.GaussianBlur(mask, (3,3), 2)

        ### only take the bottom middle 1/3 of the image
        roi = np.zeros_like(mask)
        roi[mask.shape[0]//3*2:, mask.shape[1]//3:mask.shape[1]//3*2] = 1.0
        
        mask = np.multiply(mask, roi)

        ### find centroid of mask
        M = cv2.moments(mask)
        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        return cX, cY
 

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