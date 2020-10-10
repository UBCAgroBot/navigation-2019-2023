import cv2
import numpy as np
import time
import operator
import sys
from pid import PID as pid
import functions as fn
import params
import math

class MiniContoursAlgorithm():
    # applies hsv binarization to the image
    # slices the image into horizontal strips and finds all the contours in each strip
    # determines the centroids for all the mini-contours
    # takes all theses centroids and applies hough transform to find the lines that pass through the centroids
    def __init__(self, max_vote=0):

        # smoothing kernel
        self.kernel = np.ones((5,5),np.float32)/25

        # thresholds for the color of the crop rows
        self.low_green = np.array([25, 52, 72])
        self.high_green = np.array([102, 255, 255])
        
        # random colors for drawing lines etc
        self.color1 = (255, 255, 0) #blue
        self.color2 = (200, 200, 255) #pink
        
        # parameters for HoughLinesPointSet
        self.max_vote = max_vote
        self.num_strips=60
        self.lines_max=30
        self.threshold=4
        self.min_rho=0
        self.max_rho=1000
        self.rho_step=1
        self.min_theta=-math.pi/4
        self.max_theta=math.pi/4
        self.theta_step=math.pi/180

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
        width = int(mask.shape[0]/num_strips)
        for i in range (num_strips):
            strips.append(mask[i*width:(i+1)*width, 0:mask.shape[1]])

        centroids = []
        for i, strip in enumerate(strips):
            _, contours, hierarchy = cv2.findContours(strip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            strip_centroids = []
            for contour in contours:
                M = cv2.moments(contour)
                try:
                    cX = int(M["m10"] / M["m00"])
                    strip_centroids.append((cX, strip.shape[0] * (i+0.5)))
                except:
                    pass
            centroids.append(strip_centroids)

        return centroids
    
    
    def getCenterHoughLines(self, frame, num_strips=60, lines_max=30, threshold=4, min_rho=0, max_rho=1000, rho_step=1, min_theta=-math.pi/4, max_theta=math.pi/4, theta_step=math.pi/180):
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
        
        points = np.zeros(mask.shape)
        points_vector = []
        
        for i, strip_centroid in enumerate(centroids):
            if i > int(0.3*len(centroids)):
                for centroid in strip_centroid:
                        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, self.color1, -1) 
                        
                        points_vector.append([int(centroid[0]), int(centroid[1])])


        c_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        for point in points_vector:
            try:
                points[point[1]][point[0]] = 255
            except:
                pass
        points_vector = np.array([points_vector])
        lines = cv2.HoughLinesPointSet(points_vector, lines_max=lines_max, threshold=threshold, min_rho=min_rho, max_rho=max_rho, rho_step=rho_step, min_theta=min_theta, max_theta=max_theta, theta_step=theta_step)

        point_lines = []
        if lines is not None:
            for line in lines:
                if line[0][0] > self.max_vote:
                    rho = line[0][1]
                    theta = line[0][2]
                    a = math.cos(theta)
                    b = math.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 2000*(-b)), int(y0 + 2000*(a)))
                    pt2 = (int(x0 - 2000*(-b)), int(y0 - 2000*(a)))
                    point_lines.append([line[0][0], pt1, pt2])
                    cv2.line(points, pt1, pt2, (255), 6, cv2.LINE_AA)
                    cv2.line(frame, pt1, pt2, (0,0,255), 6, cv2.LINE_AA)


        return frame, lines, point_lines

    def process_frame(self, originalframe):
        
        # original_frame: BGR frame
        # returns frame: original_frame with the lines and centroids drawn on
        frame = self.apply_filters(originalframe)
        
        frame, lines, point_lines = self.getCenterHoughLines(frame, 
                                                             num_strips=100,
                                                             lines_max=self.lines_max,
                                                             threshold=self.threshold,
                                                             min_rho=self.min_rho,
                                                             max_rho=self.max_rho,
                                                             rho_step=self.rho_step, 
                                                             min_theta=self.min_theta,
                                                             max_theta=self.max_theta,
                                                             theta_step=self.theta_step)

        return frame
        
   
