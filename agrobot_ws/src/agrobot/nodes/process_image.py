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

def process_frame(frame, last_cX):
    original_frame = frame
    height = frame.shape[0]
    width = frame.shape[1]

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray_frame, 80, 90, cv2.THRESH_BINARY_INV)
    cv2.imshow('threshold', thresh)
    cv2.waitKey(1)
    M = cv2.moments(thresh)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
    else:
        cX = last_cX
    cY = height/2

    cv2.circle(original_frame,(cX,cY), 10, (0,0,255), -1)

    return original_frame, cX

class image_converter:

  def __init__(self):
    self.centroid_location_pub = rospy.Publisher("centroid_location", Float32)
    self.last_location = 400
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/robot/camera/image_raw",Image,self.callback)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    processed_image, location = process_frame(cv_image, self.last_location)
    self.last_location = location
    cv2.imshow("Image window", processed_image)
    cv2.waitKey(1)

    self.centroid_location_pub.publish(location)

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