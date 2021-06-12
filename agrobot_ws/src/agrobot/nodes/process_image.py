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
import hough_algorithm, hough_algorithm2, center_row_algorithm
import mini_contour_algorithm
import downward_algorithm
import scanning_algorithm
from collections import deque

WIDTH = 800
LENGTH = 800

PROCESS_FRONT = True
PROCESS_DOWN = False
PROCESS_BACK = False
class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.centroid_location_pub = rospy.Publisher("centroid_location", Float32, queue_size=1)
        self.bottom_centroid = int(WIDTH/2)
        self.image_sub = rospy.Subscriber("/robot/camera/image_raw",Image,self.callback)
        self.downward_image_sub = rospy.Subscriber("/robot/downward_camera/downward_image_raw",Image,self.downward_callback)
        self.back_image_sub = rospy.Subscriber("/robot/back_camera/back_image_raw", Image, self.back_callback)
        self.h = hough_algorithm.hough_algorithm()
        self.h_back = hough_algorithm.hough_algorithm()
        self.h2 = hough_algorithm2.hough_algorithm('front')
        self.h2_back = hough_algorithm2.hough_algorithm('back')
        self.c = center_row_algorithm.contour_algorithm()
        self.m = mini_contour_algorithm.mini_contour_algorithm()
        self.last_location = int(WIDTH/2)
        self.last_centroids = deque(maxlen=10)
        self.back_intersect_point = None
        self.d = downward_algorithm.downward_algorithm()
        self.downward_image = None
        self.back_image = None
        self.scanning_algorithm = scanning_algorithm.scanning_algorithm(WIDTH, LENGTH)
        
        
        #for recording video
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter('/home/davidw0311/AgroBot/scanning_algorithm/forward_vid_for_scan.mp4',self.fourcc, 20.0, (800,800))
    
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        self.out.write(cv_image)
        # cv2.imshow("original frame", cv_image)
        
        if PROCESS_FRONT:
            # processed_image, intersect_point = self.h2.processFrame(cv_image)
            # print(self.scanning_algorithm)

            processed_image, intersect_point = self.scanning_algorithm.processFrame(cv_image, show=False)
            print(intersect_point)
            # intersect_point = intersect_point_pair[1]
        if PROCESS_BACK:
            back_processed_image, back_intersect_point = self.h2_back.processFrame(self.back_image)

        # processed_image, intersect_point = self.h2.processFrame(cv_image)
        # processed_image, intersect_point = self.c.processFrame(cv_image)
        # processed_image, lines = self.m.processFrame(cv_image, num_strips=100)
        # intersect_point = None

       
        if intersect_point is not None:
            # averaging
            # self.last_centroids.append(intersect_point[0])
            # centroid_location = sum(self.last_centroids)/len(self.last_centroids)
            # self.centroid_location_pub.publish(centroid_location)
            # self.last_location = centroid_location
            # cv2.circle(processed_image, (centroid_location, processed_image.shape[1]/2), 10, (0, 255, 255), -1)
            if PROCESS_BACK:
                if back_intersect_point is not None:
                    avg_point = (intersect_point[0] + back_intersect_point[0])/2
                    self.centroid_location_pub.publish(avg_point)
            # don't average the centroid
            self.centroid_location_pub.publish(intersect_point[0])
            self.last_location = intersect_point[0]
            cv2.circle(processed_image, (intersect_point[0], processed_image.shape[1]/2), 10, (0, 255, 255), -1)
        else:
            self.centroid_location_pub.publish(self.last_location)
            cv2.circle(processed_image, (self.last_location, processed_image.shape[1]/2), 10, (0, 255, 255), -1)

        if PROCESS_FRONT:
            cv2.imshow("Image window", processed_image)
        
        if PROCESS_DOWN:
            cv2.imshow("downward image", self.downward_image)
        
        if PROCESS_BACK:
            cv2.imshow("back image", back_processed_image)
        # for recording video
        
        cv2.waitKey(1)

    def downward_callback(self,data):
        try:
            downward_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.downward_image = self.d.processFrame(downward_cv_image)
        # downward_cv_image = self.d.processFrame(downward_cv_image)
    
    def back_callback(self,data):
        try:
            back_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.back_image = back_cv_image
        # downward_cv_image = self.d.processFrame(downward_cv_image)



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