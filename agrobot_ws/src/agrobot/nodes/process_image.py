#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('agrobot')
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import hough_algorithm

class image_converter:

  def __init__(self):
    self.centroid_location_pub = rospy.Publisher("centroid_location", Float32)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/robot/camera/image_raw",Image,self.callback)
    self.h = hough_algorithm.hough_algorithm()
    self.last_location = 100


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    processed_image, intersect_point = self.h.processFrame(cv_image)

    cv2.imshow("Image window", processed_image)
    cv2.waitKey(1)

    if intersect_point is not None:
        self.centroid_location_pub.publish(intersect_point[0])
        self.last_location = intersect_point[0]
    else:
        self.centroid_location_pub.publish(self.last_location)


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