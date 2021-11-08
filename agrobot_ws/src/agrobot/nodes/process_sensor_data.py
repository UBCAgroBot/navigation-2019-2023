#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('agrobot')
import sys
import rospy
import cv2
from std_msgs.msg import Float32, String

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from copy import copy
import os
from omegaconf import OmegaConf

from algorithms.ScanningAlgorithm import ScanningAlgorithm
from algorithms.MiniContoursAlgorithm import MiniContoursAlgorithm
from algorithms.MiniContoursDownwards import MiniContoursDownwards
class AgrobotSensors:
    ''' 
    performs image processing on camera input from agrobot
    '''
    def __init__(self):
        self.bridge = CvBridge()
        
        # self.front_x_pub = rospy.Publisher("agrobot/centroids/front_x", Float32, queue_size=5)
        # self.front_angle_pub = rospy.Publisher("agrobot/angles/front_angle", Float32, queue_size=5)

        self.vid_config = OmegaConf.load('/home/davidw0311/AgroBot/Navigation/config/video/down1.yaml')
        self.vid_config.frame_width = 400
        self.vid_config.frame_length = 400
        
        # print('\n\n\n', cv2.__version__)
        self.config = OmegaConf.load('/home/davidw0311/AgroBot/Navigation/config/algorithm/mini_contour_downward.yaml')
        self.config = OmegaConf.merge(self.config, self.vid_config)
        # self.algorithm = ScanningAlgorithm(self.config)
        self.downward_algorithm = MiniContoursDownwards(self.config)
        # self.config = OmegaConf.load('/home/davidw0311/AgroBot/Navigation/config/algorithm/mini_contour.yaml')
        # self.config = OmegaConf.merge(self.config, self.vid_config)
        # self.algorithm = MiniContoursAlgorithm(self.config)

        rospy.Subscriber("agrobot/front_camera/image", Image, self.front_camera_callback)
        rospy.Subscriber("agrobot/downward_camera/image", Image, self.downward_camera_callback)
        
        self.sensor_data_pub = rospy.Publisher("agrobot/sensors_data", String, queue_size=5)

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
        
        # processed_image, intersection_point = self.algorithm.processFrame(copy(front_image), show=False)

        processed_image, intersection_point, delta = self.downward_algorithm.processFrame(copy(downward_image), delta=True)
    

        message = String(str([delta[1][0], delta[0][0]]))
        # print(message)
        self.sensor_data_pub.publish(message)
        # front_cx, front_cy = self.get_centroid(front_image)
        
        # front_image = cv2.circle(front_image, (front_cx, front_cy), 10, (0,255,255), 1)
        # cv2.imshow('front image', front_image)
        cv2.imshow('processed', processed_image)
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