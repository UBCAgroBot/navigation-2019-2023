import cv2
import numpy as np
import math
from .MiniContoursAlgorithm import MiniContoursAlgorithm

class MiniContoursDownwards(MiniContoursAlgorithm):
    
    def __init__(self, config):
        MiniContoursAlgorithm.__init__(self,config)

    def getCenterHoughLines(self, frame, num_strips=60, lines_max=30, threshold=4, min_rho=0, max_rho=1000, rho_step=1, min_theta=-math.pi/4, max_theta=math.pi/4, theta_step=math.pi/180,
                            drawPoints=False):
        # frame: BGR frame 
        # num_strips: number of strips for centroid calculation
        # other parameters for HoughLinesPointSet
        # returns: 
        # frame: original frame with hough lines drawn on
        # lines: list of [[votes, rho, theta]] of all lines
        # point_lines: list of [votes, pt1, pt2] of all lines        
        
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), self.low_green, self.high_green)
        mask = cv2.medianBlur(mask, 9)
        centroids = self.getCentroids(mask, num_strips=num_strips)
        
        points = np.zeros(mask.shape, dtype=np.uint8)
        points_vector = []

        height, width = frame.shape[0], frame.shape[1]
        splitFactor = 0.25
        if drawPoints:
            cv2.line(frame,(int(width*splitFactor), 0), (int(width*splitFactor), height), self.color3, thickness=2)
            cv2.line(frame,(int(width*(1-splitFactor)), 0), (int(width*(1-splitFactor)), height), self.color3, thickness=2)
        for i, strip_centroid in enumerate(centroids):
            for centroid in strip_centroid:
                x,y = centroid[0], centroid[1]
                if x > width*splitFactor and x < width*(1-splitFactor):
                    # vertically split the points
                    if drawPoints:
                        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, self.color1, -1) 
                        cv2.circle(mask, (int(centroid[0]), int(centroid[1])), 3, self.color1, -1)
                    points_vector.append([int(centroid[0]), int(centroid[1])])
                else:
                    if drawPoints:
                        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, self.color3, -1)     

                        
        c_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        for point in points_vector:
            try:
                points[point[1]][point[0]] = 255
            except:
                pass

        # linear regression best fit line
        points_vector = np.array([points_vector])
        line = cv2.fitLine(points_vector, cv2.DIST_L1, 0, 0.01, 0.01)
        v1,v2,x1,x2 = np.float32(line)
        alpha = width
        p1, p2 = (int(x1-alpha*v1),int(x2-alpha*v2)), (int(x1+alpha*v1),int(x2+alpha*v2))
        cv2.line(frame, p1, p2, self.color3, thickness=3)

        # calculate center point of best fit line
        slope = v2/v1
        x = (height//2 - x2)/slope + x1
        cv2.circle(frame, (int(x), height // 2), 5, self.color1, -1)

        # reference center line and point
        centerP = (width // 2, height // 2)
        cv2.line(frame, centerP, (width // 2, 0), self.color2, thickness=3)
        cv2.circle(frame, centerP, 5, self.color2, -1)

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
        cv2.putText(frame, txt,(0,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        lines = [line]
        point_lines = []


        cv2.imshow('frame', frame)
        cv2.imshow('mask', mask)
        cv2.imshow('c_mask', c_mask)
        cv2.imshow('points', points)
        return frame, lines, point_lines