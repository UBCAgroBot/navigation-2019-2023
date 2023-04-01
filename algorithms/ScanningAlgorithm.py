import cv2
import numpy
import numpy as np
import time
import math
from algorithms.utils import Lines
from algorithms.Algorithm import Algorithm


class ScanningAlgorithm(Algorithm):
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
        self.left_x_bound = int(self.WIDTH * self.config.bounding_box_x)
        self.right_x_bound = int(self.WIDTH * (1 - self.config.bounding_box_x))
        self.upper_y_bound = int(self.HEIGHT * self.config.bounding_box_y)
        self.lower_y_bound = self.HEIGHT - 1
        self.mid_y = self.lower_y_bound - (self.lower_y_bound - self.upper_y_bound) // 2

        self.kernel = np.ones((5, 5), np.uint8)

        self.num_of_lines = config.num_of_lines

        self.lines = []

        # creates lines from the top of the horizon to the bottom
        for top_x in range(0, self.WIDTH, self.pixel_gap):
            for bottom_x in range(0, self.WIDTH, self.pixel_gap):
                line = self.create_line(top_x, self.upper_y_bound, bottom_x, self.lower_y_bound)
                self.lines.append(line)

        # creates lines from horizon to the right side
        for top_x in range(0, self.WIDTH, self.pixel_gap):
            for right_y in range(self.mid_y,
                                 self.lower_y_bound, self.pixel_gap):
                line1 = self.create_line(top_x, self.upper_y_bound, self.WIDTH - 1, right_y)
                self.lines.append(line1)

        # creates lines from horizon to the left side
        for top_x in range(0, self.WIDTH, self.pixel_gap):
            for left_y in range(self.mid_y,
                                self.lower_y_bound, self.pixel_gap):
                line = self.create_line(top_x, self.upper_y_bound, 0, left_y)
                self.lines.append(line)

        # list of all lines created
        self.lines = np.array(self.lines)

    # creates an array of x,y points for a line starting from a point on the top edge
    # extending to a point on the bottom edge
    def create_line(self, start_x, start_y, end_x, end_y):
        x_diff = abs(start_x - end_x)
        y_diff = abs(start_y - end_y)

        # keep the length constant between the two arrays
        length = int(max(x_diff, y_diff) / 5)

        x = np.linspace(start_x, end_x, length + 1).astype(int)
        y = np.linspace(start_y, end_y, length + 1).astype(int)

        # array of x, y coordinates defining the line
        line = np.stack((x, y), axis=1)

        return line

    def get_extra_content(self, frame, show):
        item1, item2 = self.process_frame(frame, show)
        maskf = frame
        return item1, item2, maskf

    def update_lower_hsv(self, next):
        self.LOW_GREEN = np.array(next)

    def update_upper_hsv(self, next):
        self.HIGH_GREEN = np.array(next)

    def process_frame(self, frame, show):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.LOWER_GREEN, self.UPPER_GREEN)
        mask = cv2.GaussianBlur(mask, (3, 3), 2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # finds the end of the crop row by using the y value of the first white pixel in the mask
        white_pixels = np.array(np.where(mask == 255))

        if len(white_pixels) == 0 or len(white_pixels[0]) == 0:
            return mask, None

        self.upper_y_bound = white_pixels[0][0]
        self.mid_y = self.lower_y_bound - (self.lower_y_bound - self.upper_y_bound) // 2

        # use this to invert the mask
        # mask = ~mask

        # array to hold (percentage, line) pairs
        # percentage is the percentage of the line that is white when overlaying mask
        # line is the (x1, y1, x2, y2) definition of the line
        pos_array = []
        neg_array = []

        # separate positive and negative sloped lines and choose the best from each group
        for line in self.lines:
            row = line[:, 0]
            col = line[:, 1]
            extracted = mask[col, row]
            if line[0][0] > line[-1][0]:
                pos_array.append((np.sum(extracted) / len(extracted), line))
            elif line[0][0] < line[-1][0]:
                neg_array.append((np.sum(extracted) / len(extracted), line))

        pos_array = np.array(pos_array)
        neg_array = np.array(neg_array)

        pos_values = pos_array[:, 0]
        neg_values = neg_array[:, 0]

        largest_pos_indices = (-pos_values).argsort()[:self.num_of_lines]
        largest_neg_indices = (-neg_values).argsort()[:self.num_of_lines]

        most_prominent_pos_lines = pos_array[:, 1][largest_pos_indices]
        most_prominent_neg_lines = neg_array[:, 1][largest_neg_indices]

        most_prominent_lines = numpy.concatenate((most_prominent_pos_lines, most_prominent_neg_lines), axis=None)

        # convert to lines as defined in Lines.py
        converted_lines = []
        for line in most_prominent_lines:
            converted_line = [line[0][0], line[0][1], line[-1][0], line[-1][1]]
            converted_lines.append(converted_line)

        intersections = Lines.get_intersections(converted_lines, 0.5)

        if not intersections:
            return frame, None

        x_points = [point[0] for point in intersections]
        y_points = [point[1] for point in intersections]
        vanishing_point = Lines.draw_vanishing_point(frame, x_points, y_points, show)

        if show:
            for line in converted_lines:
                frame = cv2.line(frame, (line[0], line[1]), (line[2], line[3]), (255, 255, 255), 1)

            # line for the middle of frame
            frame = cv2.line(frame, (self.WIDTH // 2, self.HEIGHT), (self.WIDTH // 2, 0), (0, 0, 255), 1)

            # point with x coordinate of the vanishing point and y coordinate of the end of the crop row
            frame = cv2.circle(frame, (vanishing_point[0], self.upper_y_bound), 5, (0, 255, 0), -1)

            # point in the middle of frame at midpoint between the horizon and bottom of the screen
            frame = cv2.circle(frame, (self.WIDTH // 2, self.mid_y), 5, (0, 255, 0), -1)

            # line between the two points above
            frame = cv2.line(frame, (self.WIDTH // 2, self.mid_y),
                             (vanishing_point[0], self.upper_y_bound), (0, 255, 0), 2)

        # Calculating angle from vanishing point to (self.WIDTH // 2, 0)
        angle = Lines.calculate_angle_from_v_point(vanishing_point, self.WIDTH, self.HEIGHT)

        return frame, angle

    # helper function to resize a frame mat object

    def resize(self, frame, factor):
        # resize frame to smaller size to allow faster processing
        return cv2.resize(frame, (frame.shape[1] // factor, frame.shape[0] // factor))
