import cv2
import numpy as np
import math
from .MiniContoursAlgorithm import MiniContoursAlgorithm

class MiniContoursDownwards(MiniContoursAlgorithm):
    
    def __init__(self, config):
        MiniContoursAlgorithm.__init__(self,config)
        self.speed_up = 5
        self.frame_counter = 0
        self.last_valid_frame = []
        self.end_of_row_cut_off = 0.6

    def get_center_hough_lines(self, frame, show, num_strips=60, lines_max=30, threshold=4, min_rho=0, max_rho=1000, rho_step=1, min_theta=-math.pi/4, max_theta=math.pi/4, theta_step=math.pi/180,
                            draw_points=False):
        # frame: BGR frame 
        # num_strips: number of strips for centroid calculation
        # other parameters for HoughLinesPointSet
        # returns: 
        # frame: original frame with hough lines drawn on
        # lines: list of [[votes, rho, theta]] of all lines
        # point_lines: list of [votes, pt1, pt2] of all lines        
        
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), self.low_green, self.high_green)
        mask = cv2.medianBlur(mask, 9)
        kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3,3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations = 3)
        centroids = self.getCentroids(mask, num_strips=num_strips)
        
        points = np.zeros(mask.shape, dtype=np.uint8)
        points = []

        height, width = frame.shape[0], frame.shape[1]
        split_factor = 0.25
        if draw_points:
            cv2.line(frame,(int(width*split_factor), 0), (int(width*split_factor), height), self.color3, thickness=2)
            cv2.line(frame,(int(width*(1-split_factor)), 0), (int(width*(1-split_factor)), height), self.color3, thickness=2)
        for i, strip_centroid in enumerate(centroids):
            for centroid in strip_centroid:
                x,y = centroid[0], centroid[1]
                if x > width*split_factor and x < width*(1-split_factor):
                    # vertically split the points
                    if draw_points:
                        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, self.color1, -1) 
                        cv2.circle(mask, (int(centroid[0]), int(centroid[1])), 3, self.color1, -1)
                    points.append([int(centroid[0]), int(centroid[1])])
                else:
                    if draw_points:
                        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, self.color3, -1)     

                        
        c_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) 

        for point in points:
            try:
                points[point[1]][point[0]] = 255
            except:
                pass

        # linear regression best fit line
        points = np.array([points])
        if len(points[0]) <= 0:
            return frame, points[0], ([0],[0])
        line = cv2.fitLine(points, cv2.DIST_L1, 0, 0.01, 0.01)
        v1,v2,x1,x2 = np.float32(line)
        alpha = width
        p1, p2 = (int(x1-alpha*v1),int(x2-alpha*v2)), (int(x1+alpha*v1),int(x2+alpha*v2))
        cv2.line(frame, p1, p2, self.color3, thickness=3)

        # calculate center point of best fit line
        slope = v2/v1
        x = (height//2 - x2)/slope + x1
        cv2.circle(frame, (int(x), height // 2), 5, self.color1, -1)

        # reference center line and point
        center_point = (width // 2, height // 2)
        cv2.line(frame, center_point, (width // 2, 0), self.color2, thickness=3)
        cv2.circle(frame, center_point, 5, self.color2, -1)

        # calculate angle
        up = [0,-1]
        down = [0,1]
        dir = [v1,v2]
        norm = np.linalg.norm(up) * np.linalg.norm(dir)
        u_dp, d_dp = np.dot(up,dir)/norm, np.dot(down,dir)/norm
        u_angle, d_angle = np.arccos(u_dp), np.arccos(d_dp)
        angle = min(u_angle,d_angle)
        deg = angle*180/np.pi
        direction = "left" if u_angle > d_angle else "right"
        sign = 1 if u_angle > d_angle else -1

        txt = "angle: " + str(np.round(deg[0], 3)) + " deg " + direction + " x offset: " + str(width//2-int(x)) + " pixels"
        cv2.putText(frame, txt,(0,100), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 2, cv2.LINE_AA)

        lines = [line]
        points = points[0]

        deltas = (sign*deg, width//2 - x)
        if show:
            # cv2.imshow('frame', frame)
            cv2.imshow('mask', mask)
            cv2.imshow('points', points)
        return frame, points, deltas
    
    def check_end_of_row(self, frame, points):

        height, width = frame.shape[0], frame.shape[1]
        if len(points) <= 0:
            end_of_row = True
            cv2.putText(frame, "end of row detected",(width // 2 , height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 2, cv2.LINE_AA)
            return frame, end_of_row

        avg = np.sum(points, axis=0) / len(points)
        cv2.circle(frame, (avg[0], avg[1]), 5, self.color1, -1) 

        end_of_row = False
        if avg[1] > height*self.end_of_row_cut_off:
            end_of_row = True
            cv2.putText(frame, "end of row detected",(width // 2 , height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 2, cv2.LINE_AA)

        return frame, end_of_row

    
    def process_frame(self, original_frame, show, delta=False):
        # original_frame: BGR frame
        # returns frame: original_frame with the lines and centroids drawn on

        # increment frame counter
        self.frame_counter += 1

        # check if time to process a frame
        if self.frame_counter % self.speed_up != 0:
            if delta:
                return self.last_valid_frame, True, False, (0,0)
            else:
                return self.last_valid_frame, True, False

        # call processing functions
        frame = self.apply_filters(original_frame)
        frame, points, deltas = self.get_center_hough_lines(frame, show)

        frame, end_of_row = self.check_end_of_row(frame, points)

        # store frames
        self.last_valid_frame = frame

        if delta: #commented out False
            return frame, False, end_of_row, deltas
        else:
            return frame, False, end_of_row