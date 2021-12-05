#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('agrobot')
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import os

HEIGHT = 400
WIDTH = 400

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("agrobot/front_camera/image",Image,self.front_callback)
        self.downward_image_sub = rospy.Subscriber("/agrobot/downward_camera/image",Image,self.downward_callback)
       
        self.left_image_sub = rospy.Subscriber("/agrobot/left_side_camera/image", Image, self.left_callback)
        self.right_image_sub = rospy.Subscriber("/agrobot/right_side_camera/image", Image, self.right_callback)
    
        self.front_image = np.zeros((WIDTH, HEIGHT))
        self.downward_image = np.zeros((WIDTH, HEIGHT))
        self.left_image = np.zeros((WIDTH, HEIGHT))
        self.right_image = np.zeros((WIDTH, HEIGHT))

        
        #for recording video
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')


        vid_folder = '/home/davidw0311/AgroBot/videos/end_of_row_turning/'
        if not os.path.exists(vid_folder):
            os.makedirs(vid_folder)
        self.front_recorder = cv2.VideoWriter(vid_folder+'front.mp4',self.fourcc, 10.0, (WIDTH,HEIGHT))
        self.left_recorder = cv2.VideoWriter(vid_folder+'left.mp4',self.fourcc, 10.0, (WIDTH,HEIGHT))
        self.right_recorder = cv2.VideoWriter(vid_folder+'right.mp4',self.fourcc, 10.0, (WIDTH,HEIGHT))
        self.downward_recorder = cv2.VideoWriter(vid_folder+'down.mp4',self.fourcc, 10.0, (WIDTH,HEIGHT))
    
    def front_callback(self,data):
        try:
            self.front_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("front image", self.front_image)
        cv2.imshow("left image", self.left_image)
        cv2.imshow("right image", self.right_image)
        cv2.imshow("downward image", self.downward_image)
        cv2.waitKey(1)

        # for saving video
        self.front_recorder.write(self.front_image)
        self.left_recorder.write(self.left_image)
        self.right_recorder.write(self.right_image)
        self.downward_recorder.write(self.downward_image)
        
        

    def downward_callback(self,data):
        try:
            downward_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.downward_image = downward_cv_image


    def left_callback(self,data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        

    def right_callback(self,data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)