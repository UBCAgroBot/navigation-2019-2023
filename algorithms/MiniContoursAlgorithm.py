import cv2
import numpy as np
import time
import operator
import sys
import math

class MiniContoursAlgorithm:
    # applies hsv binarization to the image
    # slices the image into horizontal strips and finds all the contours in each strip
    # determines the centroids for all the mini-contours
    # takes all theses centroids and applies hough transform to find the lines that pass through the centroids
    def __init__(self, config):
        
        self.config = config
        # smoothing kernel
        self.kernel = np.ones((5,5),np.float32)/25
        self.morphology_kernel = np.ones((9,9),np.float32)

        # thresholds for the color of the crop rows
        self.low_green = np.array(config.low_green)
        self.high_green = np.array(config.high_green)

        # random colors for drawing lines etc
        self.color_1 = (255, 255, 0) #blue
        self.color_2 = (200, 200, 255) #pink
        self.color_3 = (0,0,255) #red (removed points)
        self.contour_color = (0, 129, 255)

        # cutOff for points
        self.cut_off_height_factor = self.config.cut_off_factor
        
        # parameters for HoughLinesPointSet
        self.max_vote = self.config.max_vote
        self.num_strips=self.config.num_strips
        self.lines_max=self.config.lines_max
        self.threshold=self.config.threshold
        self.min_rho=self.config.min_rho
        self.max_rho=self.config.max_rho
        self.rho_step=self.config.rho_step
        self.min_theta=-math.pi/4
        self.max_theta=math.pi/4
        self.theta_step=math.pi/180

    def apply_filters(self, original_frame):
        # orginal_frame: BGR frame 
        # returns frame: filtered BGR frame

        frame = cv2.filter2D(original_frame, -1, self.kernel)
        frame = cv2.bilateralFilter(frame, 9, 10, 75)    

        return frame

    def get_centroids(self, mask, num_strips):

        # mask is a binary mask of the image
        # number of strips is the number of strips to divide the image into for finding centroids
        # returns list [(x1, y1), (x2, y2), ...] of all the centroids obtained
        
        strips = []
        width = int(mask.shape[0]/num_strips)
        for i in range (num_strips):
            strips.append(mask[i*width:(i+1)*width, 0:mask.shape[1]])
            # cv2.line(mask, (0,i*width), (mask.shape[1],i*width), (255,0,255), 1, cv2.LINE_AA)

        centroids = []
        for i, strip in enumerate(strips):
            contours, hierarchy = cv2.findContours(strip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    
    
    def get_center_hough_lines(self, frame, num_strips=60, lines_max=30, threshold=4, min_rho=0, max_rho=1000, rho_step=1, min_theta=-math.pi/4, max_theta=math.pi/4, theta_step=math.pi/180):
        # frame: BGR frame 
        # num_strips: number of strips for centroid calculation
        # other parameters for HoughLinesPointSet
        # returns: 
        # frame: original frame with hough lines drawn on
        # lines: list of [[votes, rho, theta]] of all lines
        # point_lines: list of [votes, pt1, pt2] of all lines        
        
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), self.low_green, self.high_green)
        mask = cv2.medianBlur(mask, 9)
        # mask = cv2.GaussianBlur(mask, (9,9), 10)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morphology_kernel)
        centroids = self.get_centroids(mask, num_strips=num_strips)
        
        points = np.zeros(mask.shape, dtype=np.uint8)
        # points = cv2.Mat.zeros(mask.shape[0], mask.shape[1], cv.CV_8UC3)
        points_vector = []
        
        # ret, thresh = cv2.threshold(mask, 0, 254, 0)
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cnt = contours
        # cv2.drawContours(frame, cnt, -1, self.contour_color, 3)
        # cv2.fillPoly(frame, pts=cnt, color=(0, 255, 0))
        # for c in cnt:
        #     if cv2.contourArea(c) > 3000:
        #         ellipse = cv2.fitEllipse(c)
        #         cv2.ellipse(frame, ellipse, (255, 255, 255), 2)
                


        height, width = frame.shape[0], frame.shape[1]
        split_factor = 10
        segmented_points = [[] for _ in range(split_factor)]
        cut_off_height = (int)(self.cut_off_height_factor * height)
        cv2.line(frame, (0, cut_off_height), (width//2, 0), self.color_3)
        cv2.line(frame, (width, cut_off_height), (width//2, 0), self.color_3)
        for i, strip_centroid in enumerate(centroids):
            if i > int(0.3*len(centroids)):

                for centroid in strip_centroid:
                    x,y = centroid[0], centroid[1]
                    if y > -(cut_off_height/(width//2))*x + cut_off_height and y > (cut_off_height/(width//2))*x - cut_off_height:

                        # vertically split the points
                        idx = int(x / width * split_factor)
                        segmented_points[idx].append([int(x),int(y)])


                        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, self.color_1, -1) 
                        cv2.circle(mask, (int(centroid[0]), int(centroid[1])), 3, self.color_1, -1)
                        points_vector.append([int(centroid[0]), int(centroid[1])])
                        
                    else:
                        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, self.color_3, -1) 
                        
        c_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        for point in points_vector:
            try:
                points[point[1]][point[0]] = 255
            except:
                pass
        points_vector = np.array([points_vector])
        lines = cv2.HoughLinesPointSet(
            points_vector, 
            lines_max=lines_max, 
            threshold=threshold, 
            min_rho=min_rho, 
            max_rho=max_rho, 
            rho_step=rho_step, 
            min_theta=min_theta, 
            max_theta=max_theta, 
            theta_step=theta_step
        )

        point_lines = []
        # if lines is not None:
        #     for l in lines:
        #         vx, vy, px, py = l[0],l[1],l[2],l[3]
        #         p2x = px + width*vx
        #         p2y = py + width*vy
        #         p1,p2 = (px,py), (p2x, p2y)

        #         cv2.line(frame, p1, p2, self.color_2, 6, cv2.LINE_AA)

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


        cv2.imshow('frame', frame)
        cv2.imshow('mask', mask)
        cv2.imshow('c_mask', c_mask)
        cv2.imshow('points', points)


        return frame, lines, point_lines


    def process_frame(self, original_frame, num_strips=60, show=False):
        
        # original_frame: BGR frame
        # returns frame: original_frame with the lines and centroids drawn on
        frame = self.apply_filters(original_frame)
        
        frame, lines, point_lines = self.get_center_hough_lines(frame, 
                                                             num_strips=num_strips,
                                                             lines_max=self.lines_max,
                                                             threshold=self.threshold,
                                                             min_rho=self.min_rho,
                                                             max_rho=self.max_rho,
                                                             rho_step=self.rho_step, 
                                                             min_theta=self.min_theta,
                                                             max_theta=self.max_theta,
                                                             theta_step=self.theta_step)

        # cv2.imshow('frame', frame)

        return frame, point_lines