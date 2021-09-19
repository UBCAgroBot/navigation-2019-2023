import cv2
import numpy as np
import time
from algorithms.utils import Lines

class ScanningAlgorithm:
    def __init__(self, config):
        self.config=config
        self.LENGTH = config['frame_length']
        self.WIDTH = config['frame_width']

        self.LOWER_GREEN = np.array(config['lower_hsv_threshold'])
        self.UPPER_GREEN = np.array(config['upper_hsv_threshold'])

        self.blur_kernel=config['gaussian_blur_kernel']
        self.morph_kernel=np.ones((config['morph_kernel'],config['morph_kernel']), np.uint8)

        self.num_of_lines = config['num_of_lines']
        # gaussian blur / dilate
        # generate_lines_time = time.time()

        self.lines = []
        for top_x in range(0, self.WIDTH, 30):
            for bottom_x in range(0, self.WIDTH, 30):
                line = self.create_line(top_x, 0, bottom_x, self.LENGTH-1)
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


    
    # print('elapsed time to generate lines', time.time()-generate_lines_time)

    def processFrame(self, frame, show=False):
        # cv2.imshow('original frame', frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Filter image and allow only shades of green to pass
        mask = cv2.inRange(hsv, self.LOWER_GREEN, self.UPPER_GREEN)
        # Apply gaussian blur (can be removed)
        mask = cv2.GaussianBlur(mask, (self.blur_kernel,self.blur_kernel), 2)
        # mask = cv2.dilate(mask, kernel, iterations = 1)
        # mask = cv2.erode(mask, kernel, iterations= 1)
        # print(mask.shape)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
        
        # mask = ~mask
        # array to hold (percentage, line) pairs
        # percentage is the percentage of the line that is white when overlaying mask
        # line is the (x1, y1, x2, y2) definition of the line
        lines_array = []

        # find_lines_time = time.time()

        for line in self.lines:
            row = line[:,0]
            col = line[:,1]
            extracted = mask[col, row]
            lines_array.append((np.sum(extracted)/len(extracted), line))
        lines_array = np.array(lines_array)

        # finds the lines 
        values = lines_array[:, 0]
        largest_indices = (-values).argsort()[:self.num_of_lines]
        most_prominent_lines = lines_array[:, 1][largest_indices]

        # print('time to find lines', time.time()-find_lines_time)

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
 
