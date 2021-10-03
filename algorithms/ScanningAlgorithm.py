import cv2
import numpy as np
import time
from algorithms.utils import Lines

class ScanningAlgorithm(object):
    def __init__(self, config):
        
        self.config = config
        self.WIDTH = config.frame_width
        self.HEIGHT = config.frame_length
        # filter for corn
        # self.LOWER_GREEN = np.array([17, 26, 0])
        # self.UPPER_GREEN = np.array([93, 255, 255])

        # filter for wheat
        self.LOWER_GREEN = np.array(config.lower_hsv_threshold)
        self.UPPER_GREEN = np.array(config.upper_hsv_threshold)
        
        self.pixel_gap = config.pixel_gap
        self.left_x_bound = int(self.WIDTH*self.config.bounding_box_x)
        self.right_x_bound = int(self.WIDTH*(1-self.config.bounding_box_x))
        self.upper_y_bound = int(self.HEIGHT*self.config.bounding_box_y)
        self.lower_y_bound = self.HEIGHT-1

        self.kernel=np.ones((5,5), np.uint8)

        self.num_of_lines = config.num_of_lines

        self.lines = []

        # The following loops create lines, 420 is hardcoded horizon that may need to 
        # be changed. Loops can be mixed and matched for different sets of lines.

        # creates lines from the top of the horizon (in this case 420) to the bottom
        for top_x in range(self.left_x_bound,self.right_x_bound, self.pixel_gap):
            for bottom_x in range(self.left_x_bound,self.right_x_bound, self.pixel_gap):
                line = self.create_line(top_x, self.upper_y_bound, bottom_x, self.lower_y_bound)
                self.lines.append(line)

        # creates lines from left side to right side
        for left_y in range(self.upper_y_bound, self.lower_y_bound, self.pixel_gap):
            for right_y in range(self.upper_y_bound, self.lower_y_bound, self.pixel_gap):
                line = self.create_line(self.left_x_bound, left_y, self.right_x_bound, right_y)
                self.lines.append(line)

        # creates lines from horizon to the right side
        for top_x in range(self.left_x_bound, self.right_x_bound, self.pixel_gap):
            for right_y in range(self.upper_y_bound, self.lower_y_bound, self.pixel_gap):
                line1 = self.create_line(top_x, self.upper_y_bound, self.right_x_bound, right_y)

                self.lines.append(line1)

        # creates lines from horizon to the left side
        for top_x in range(self.left_x_bound, self.right_x_bound, self.pixel_gap):
            for left_y in range(self.upper_y_bound, self.lower_y_bound, self.pixel_gap):
                line = self.create_line(top_x, self.upper_y_bound, self.left_x_bound, left_y)
                self.lines.append(line)

        self.lines = np.array(self.lines)

    # creates an array of x,y points for a line starting from a point on the top edge extedning to a point on the bottom edge
    def create_line(self, start_x, start_y, end_x, end_y):
        x_diff = abs(start_x - end_x)
        y_diff = abs(start_y - end_y)
        
        # keep the length constant between the two arrays
        length = int(max(x_diff, y_diff) / 5)

        x = np.linspace(start_x, end_x, length+1).astype(int)
        y = np.linspace(start_y, end_y, length+1).astype(int)
        
        # array of x, y coordinates defining the line
        line = np.stack((x, y), axis=1)

        return line


    def processFrame(self, frame, show=False):
        cv2.imshow('original frame', frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  
        mask = cv2.inRange(hsv, self.LOWER_GREEN, self.UPPER_GREEN)
        mask = cv2.GaussianBlur(mask, (3,3), 2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # use this to invert the mask
        # mask = ~mask


        # array to hold (percentage, line) pairs
        # percentage is the percentage of the line that is white when overlaying mask
        # line is the (x1, y1, x2, y2) definition of the line
        lines_array = []

        for line in self.lines:
            row = line[:,0]
            col = line[:,1]
            extracted = mask[col, row]
            lines_array.append((np.sum(extracted)/len(extracted), line))
        lines_array = np.array(lines_array)

        values = lines_array[:, 0]
        largest_indices = (-values).argsort()[:self.num_of_lines]
        most_prominent_lines = lines_array[:, 1][largest_indices]

        # convert to lines as defined in Lines.py
        converted_lines = []
        for line in most_prominent_lines:
            converted_line = [line[0][0], line[0][1], line[-1][0], line[-1][1]]
            converted_lines.append(converted_line)
        
        intersections, points = Lines.getIntersections(converted_lines)
        vanishing_point = Lines.drawVanishingPoint(frame, points)
        for line in converted_lines:
            frame = cv2.line(frame, (line[0],line[1]), (line[2], line[3]), (255,255,255), 1) 


        if show:
            cv2.imshow('after scanning algorithm', frame)
            cv2.imshow('mask', mask)

        # print('vanishing Point: ', vanishing_point)
        return frame, vanishing_point

    # helper function to resize a frame mat object
    def resize(self, frame, factor):
        # resize frame to smaller size to allow faster processing
        return cv2.resize(frame, (int(frame.shape[1] / factor), int(frame.shape[0] / factor)))
 
