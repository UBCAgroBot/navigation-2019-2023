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

HEIGHT = 800
WIDTH = 800

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/robot/camera/image_raw",Image,self.callback)
        self.downward_image_sub = rospy.Subscriber("/robot/downward_camera/downward_image_raw",Image,self.downward_callback)
        self.back_image_sub = rospy.Subscriber("/robot/back_camera/back_image_raw", Image, self.back_callback)
        self.front_left_image_sub = rospy.Subscriber("/robot/front_left_camera/front_left_image_raw", Image, self.front_left_callback)
        self.front_right_image_sub = rospy.Subscriber("/robot/front_right_camera/front_right_image_raw", Image, self.front_right_callback)
        self.back_left_image_sub = rospy.Subscriber("/robot/back_left_camera/back_left_image_raw", Image, self.back_left_callback)
        self.back_right_image_sub = rospy.Subscriber("/robot/back_right_camera/back_right_image_raw", Image, self.back_right_callback)
       
        self.downward_image = np.zeros((WIDTH, HEIGHT))
        self.back_image = np.zeros((WIDTH, HEIGHT))
        self.front_left_image = np.zeros((WIDTH, HEIGHT))
        self.front_right_image = np.zeros((WIDTH, HEIGHT))
        self.back_left_image = np.zeros((WIDTH, HEIGHT))
        self.back_right_image = np.zeros((WIDTH, HEIGHT))
        
        #for recording video
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        self.front_recorder = cv2.VideoWriter('/home/davidw0311/AgroBot/videos/turning/front.mp4',self.fourcc, 20.0, (WIDTH,HEIGHT))
        self.back_recorder = cv2.VideoWriter('/home/davidw0311/AgroBot/videos/turning/back.mp4',self.fourcc, 20.0, (WIDTH,HEIGHT))
        self.front_left_recorder = cv2.VideoWriter('/home/davidw0311/AgroBot/videos/turning/front_left.mp4',self.fourcc, 20.0, (WIDTH,HEIGHT))
        self.front_right_recorder = cv2.VideoWriter('/home/davidw0311/AgroBot/videos/turning/front_right.mp4',self.fourcc, 20.0, (WIDTH,HEIGHT))
        self.back_left_recorder = cv2.VideoWriter('/home/davidw0311/AgroBot/videos/turning/back_left.mp4',self.fourcc, 20.0, (WIDTH,HEIGHT))
        self.back_right_recorder = cv2.VideoWriter('/home/davidw0311/AgroBot/videos/turning/back_right.mp4',self.fourcc, 20.0, (WIDTH,HEIGHT))
        self.downward_recorder = cv2.VideoWriter('/home/davidw0311/AgroBot/videos/turning/downward.mp4',self.fourcc, 20.0, (WIDTH,HEIGHT))
    
    def callback(self,data):
        try:
            front_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("front image", front_image)
        cv2.imshow("front left image", self.front_left_image)
        cv2.imshow("front right image", self.front_right_image)
        cv2.imshow("back image", self.back_image)
        cv2.imshow("back left image", self.back_left_image)
        cv2.imshow("back right image", self.back_right_image)
        cv2.imshow("downward image", self.downward_image)

        # for saving video
        self.front_recorder.write(front_image)
        self.front_left_recorder.write(self.front_left_image)
        self.front_right_recorder.write(self.front_right_image)
        self.front_left_recorder.write(self.front_left_image)
        self.back_left_recorder.write(self.back_left_image)
        self.back_right_recorder.write(self.back_right_image)
        self.downward_recorder.write(self.downward_image)
        self.back_recorder.write(self.back_image)
        
        cv2.waitKey(1)

    def downward_callback(self,data):
        try:
            downward_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.downward_image = downward_cv_image
    
    def back_callback(self,data):
        try:
            back_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.back_image = back_cv_image

    def front_left_callback(self,data):
        try:
            front_left_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.front_left_image = front_left_cv_image

    def front_right_callback(self,data):
        try:
            front_right_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.front_right_image = front_right_cv_image

    def back_left_callback(self,data):
        try:
            back_left_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.back_left_image = back_left_cv_image

    def back_right_callback(self,data):
        try:
            back_right_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.back_right_image = back_right_cv_image




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