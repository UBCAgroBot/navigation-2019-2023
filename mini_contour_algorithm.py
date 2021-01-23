<<<<<<< HEAD
import cv2
import numpy as np
import time
import operator
import sys
=======
from cv2 import cv2
from numpy import array, ones, float32, zeros
>>>>>>> origin/RithinBranch
import math
import Lines

<<<<<<< HEAD
class mini_contour_algorithm():
=======

class MiniContoursAlgorithm:
>>>>>>> origin/RithinBranch
    # applies hsv binarization to the image
    # slices the image into horizontal strips and finds all the contours in each strip
    # determines the centroids for all the mini-contours
    # takes all theses centroids and applies hough transform to find the lines that pass through the centroids
    def __init__(self, max_vote=0):

        # smoothing kernel
        self.kernel = ones((5, 5), float32) / 25

        # thresholds for the color of the crop rows
        self.low_green = array([25, 52, 72])
        self.high_green = array([102, 255, 255])

        # random colors for drawing lines etc
        self.color1 = (255, 255, 0)  # blue
        self.color2 = (200, 200, 255)  # pink

        # parameters for HoughLinesPointSet
        self.max_vote = max_vote
        self.num_strips = 60
        self.lines_max = 30
        self.threshold = 4
        self.min_rho = 0
        self.max_rho = 1000
        self.rho_step = 1
        self.min_theta = -math.pi / 4
        self.max_theta = math.pi / 4
        self.theta_step = math.pi / 180

    def apply_filters(self, originalframe):
        # orginal_frame: BGR frame 
        # returns frame: filtered BGR frame

        frame = cv2.filter2D(originalframe, -1, self.kernel)
        frame = cv2.bilateralFilter(frame, 9, 10, 75)

        return frame

    def getCentroids(self, mask, num_strips):

        # mask is a binary mask of the image
        # number of strips is the number of strips to divide the image into for finding centroids
        # returns list [(x1, y1), (x2, y2), ...] of all the centroids obtained

        strips = []
        width = int(mask.shape[0] / num_strips)
        for i in range(num_strips):
            strips.append(mask[i * width:(i + 1) * width, 0:mask.shape[1]])

        centroids = []
        for i, strip in enumerate(strips):
<<<<<<< HEAD
=======
            # the below line works for some versions of cv2, add an "_," to the start if it receives 3 values
>>>>>>> origin/RithinBranch
            contours, hierarchy = cv2.findContours(strip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            strip_centroids = []
            for contour in contours:
                M = cv2.moments(contour)
                try:
                    cX = int(M["m10"] / M["m00"])
                    strip_centroids.append((cX, strip.shape[0] * (i + 0.5)))
                except:
                    pass
            centroids.append(strip_centroids)

        return centroids

    def getCenterHoughLines(self, frame, num_strips=60, lines_max=30, threshold=4, min_rho=0, max_rho=1000, rho_step=1,
                            min_theta=-math.pi / 4, max_theta=math.pi / 4, theta_step=math.pi / 180):
        # frame: BGR frame 
        # num_strips: number of strips for centroid calculation
        # other parameters for HoughLinesPointSet
        # returns: 
        # frame: original frame with hough lines drawn on
        # lines: list of [[votes, rho, theta]] of all lines
        # point_lines: list of [votes, pt1, pt2] of all lines        

        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), self.low_green, self.high_green)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        centroids = self.getCentroids(mask, num_strips=num_strips)

        points = zeros(mask.shape)
        points_vector = []

        for i, strip_centroid in enumerate(centroids):
            if i > int(0.3 * len(centroids)):
                for centroid in strip_centroid:
                    cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, self.color1, -1)

                    points_vector.append([int(centroid[0]), int(centroid[1])])

        cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        for point in points_vector:
            try:
                points[point[1]][point[0]] = 255
            except:
                pass
        points_vector = array([points_vector])
        lines = cv2.HoughLinesPointSet(points_vector, lines_max=lines_max, threshold=threshold, min_rho=min_rho,
                                       max_rho=max_rho, rho_step=rho_step, min_theta=min_theta, max_theta=max_theta,
                                       theta_step=theta_step)

        return frame, lines

    def processFrame(self, originalframe):

        # original_frame: BGR frame
        # returns frame: original_frame with the lines and centroids drawn on
        frame = self.apply_filters(originalframe)

        frame, lines = self.getCenterHoughLines(frame,
                                                             num_strips=100,
                                                             lines_max=self.lines_max,
                                                             threshold=self.threshold,
                                                             min_rho=self.min_rho,
                                                             max_rho=self.max_rho,
                                                             rho_step=self.rho_step,
                                                             min_theta=self.min_theta,
                                                             max_theta=self.max_theta,
                                                             theta_step=self.theta_step)

<<<<<<< HEAD
        cv2.imshow('frame', frame)

        return frame
        
   
=======
        # draw detected lines on a frame
        lineimg = Lines.drawLinesOnFrame(lines, originalframe.copy())

        # Calculate intersection points
        intersections, points = Lines.getIntersections(lines)

        # Draw vanishing point
        Lines.drawVanishingPoint(lineimg, points)

        cv2.imshow('mini contour algorithm', lineimg)
        return lineimg

>>>>>>> origin/RithinBranch
