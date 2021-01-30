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
import hough_algorithm
from collections import deque

class image_converter:

    def __init__(self):
        self.centroid_location_pub = rospy.Publisher("centroid_location", Float32)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/robot/camera/image_raw",Image,self.callback)
        self.downward_image_sub = rospy.Subscriber("/robot/downward_camera/downward_image_raw",Image,self.downward_callback)
        self.h = hough_algorithm.hough_algorithm()
        self.last_location = 100
        self.downward_image = None
        self.collisions_pub = rospy.Publisher("corner_collisions", String, queue_size = 10)
        self.last_centroids = deque(maxlen=10)
    
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("original frame", cv_image)
        cv2.waitKey(1)

        processed_image, intersect_point = self.h.processFrame(cv_image)

        
        self.process_downward_image()

       
        if intersect_point is not None:
            self.last_centroids.append(intersect_point[0])
            centroid_location = sum(self.last_centroids)/len(self.last_centroids)
            self.centroid_location_pub.publish(centroid_location)
            self.last_location = centroid_location
            cv2.circle(processed_image, (centroid_location, processed_image.shape[1]/2), 10, (0, 255, 255), -1)
        else:
            self.centroid_location_pub.publish(self.last_location)
            cv2.circle(processed_image, (self.last_location, processed_image.shape[1]/2), 10, (0, 255, 255), -1)

        cv2.imshow("Image window", processed_image)
        cv2.waitKey(1)

    def downward_callback(self,data):
        try:
            downward_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.downward_image = downward_cv_image

    def process_downward_image(self):
        image = self.downward_image
        width, height, channels = image.shape

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        LOWER_GREEN = np.array([31, 43, 23])
        UPPER_GREEN = np.array([255, 255, 100])
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        
        kernel = np.ones((21,21),np.uint8)
        blur = cv2.GaussianBlur(mask,(31,31), 0)
        closed = cv2.morphologyEx(blur, cv2.MORPH_CLOSE, kernel)
        
        

        # x,y = np.nonzero(mask)
        # m,b = np.polyfit(x,y,1)
        # start_point = (int(b), 0)
        # end_point = (int(m*800 + b), 800)
        # image = cv2.line(image, start_point, end_point, (255,0,0), 4)

        cv2.imshow('mask', closed)

        def cut_rectangle_mask(c1, c2):
            return closed[c1[1]:c2[1],c1[0]:c2[0]]
        def get_coverage(corner):
            return np.sum(corner)/255/(corner.shape[0]*corner.shape[1])


        front_left_c1, front_left_c2 = (int(width*0.15), int(height*0.05)), (int(width*0.3),int(height*0.25)) 
        front_left = cut_rectangle_mask(front_left_c1, front_left_c2)
        cv2.rectangle(image, front_left_c1, front_left_c2, (0, 255, 0), 3)

        front_right_c1, front_right_c2 = (int(width*0.7), int(height*0.05)), (int(width*0.85),int(height*0.25)) 
        front_right = cut_rectangle_mask(front_right_c1, front_right_c2)
        cv2.rectangle(image, front_right_c1, front_right_c2, (0, 255, 0), 3)

        back_left_c1, back_left_c2 = (int(width*0.15), int(height*0.75)), (int(width*0.3),int(height*0.95)) 
        back_left = cut_rectangle_mask(back_left_c1, back_left_c2)
        cv2.rectangle(image, back_left_c1, back_left_c2, (0, 255, 0), 3)

        back_right_c1, back_right_c2 = (int(width*0.7), int(height*0.75)), (int(width*0.85),int(height*0.95)) 
        back_right = cut_rectangle_mask(back_right_c1, back_right_c2)
        cv2.rectangle(image, back_right_c1, back_right_c2, (0, 255, 0), 3)

        collisions = [0,0,0,0]
        if get_coverage(front_left) > 0.2:
            cv2.rectangle(image, front_left_c1, front_left_c2, (0, 0, 255), 3)
            collisions[0] = 1
        if get_coverage(front_right) > 0.2:
            cv2.rectangle(image, front_right_c1, front_right_c2, (0, 0, 255), 3)
            collisions[1] = 1
        if get_coverage(back_left) > 0.2:
            cv2.rectangle(image, back_left_c1, back_left_c2, (0, 0, 255), 3)
            collisions[2] = 1
        if get_coverage(back_right) > 0.2:
            cv2.rectangle(image, back_right_c1, back_right_c2, (0, 0, 255), 3)
            collisions[3] = 1
        
        
        cv2.imshow('front left', front_left)
        cv2.imshow('front right', front_right)
        cv2.imshow('back left', back_left)
        cv2.imshow('back right', back_right)

        

        # if get_coverage(front_left) > 0.2:
        #     rectangle((dl*5, dl*1), (dl*8, dl*4))
        # if get_coverage(front_right) > 0.2:
        #     rectangle((dl*25, dl*1), (dl*28, dl*4))
        # if get_coverage(back_left) > 0.2:
        #     rectangle((dl*5, dl*1), (dl*8, dl*4))
        # if get_coverage(front_left) > 0.2:
        #     rectangle((dl*5, dl*1), (dl*8, dl*4))

        cv2.imshow('downward image', image)
        cv2.waitKey(1)
        self.collisions_pub.publish(str(collisions))


    

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