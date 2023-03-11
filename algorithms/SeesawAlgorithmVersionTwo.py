import math
import cv2 as cv
import numpy
import numpy as np
from algorithms.Algorithm import Algorithm
from helper_scripts.change_res import change_res


class SeesawAlgorithmVersionTwo(Algorithm):

    def __init__(self, config):
        """
        Sets seesaw algorithm configurations
        :param config: config params
        """
        # masking range for green
        self.LOW_GREEN = np.array(config.lower_hsv_threshold)
        self.HIGH_GREEN = np.array(config.upper_hsv_threshold)

        # filtering parameters
        self.averaging_kernel_size = config.averaging_kernel_size
        self.gauss_kernel_size = list(map(int, config.gauss_kernel_size.split(',')))
        self.dilate_kernel_size = config.dilate_kernel_size
        self.sigma_x = config.sigma_x

        # dimensions
        self.HEIGHT = int(config.frame_length)
        self.WIDTH = int(config.frame_width)

        # visual parameters
        self.BAR_HEIGHT = config.bar_height
        self.RES_FACTOR = config.resolution_factor

    def process_frame(self, frame, show):
        """
        Divides screen into horizontal strips and find average location of green for each strip.
        Draw the best fit line based the average location,
        then calculate the angle between the best fit line and a horizontal line.
        :param frame: current frame (mat)
        :param show: show/hide frames on screen for debugging
        :type show: bool
        :return: processed frame (mat), angle [-90, 90]
        """

        frame = change_res(frame, self.RES_FACTOR)
        black_frame, average_points = self.plot_points(frame)
        average_points = np.array(average_points)

        if average_points.any():
            """get best fit line for centre points"""
            [vx, vy, x, y] = cv.fitLine(average_points, cv.DIST_L2, 0, 0.01, 0.01)
            x1 = int(x - vx * self.WIDTH)
            x2 = int(x + vx * self.WIDTH)
            y1 = int(y - vy * self.HEIGHT)
            y2 = int(y + vy * self.HEIGHT)
            black_frame = cv.line(black_frame, (x1, y1), (x2, y2), (0, 255, 255), 9)

            # calculate angle
            if y1 - y2 != 0:
                angle = round(math.degrees(math.atan(int(x2 - x1) / int(y1 - y2))), 1)
            else:
                angle = None
        else:
            angle = None

        return black_frame, angle

    def plot_points(self, frame):
        """
        Divides screen into horizontal strips, and find average location of green for each strip.
        :param frame: current frame (mat)
        :return: processed frame (mat), list of centre points
        """

        bar_height = int(self.BAR_HEIGHT)
        mask = self.create_binary_mask(frame)

        square_low = 0
        square_high = bar_height

        black_frame = frame

        centre_points = []

        while square_low < self.HEIGHT:
            points = []

            seg = mask[int(square_low) + 1:int(square_high), 0:self.WIDTH]

            condition = (seg == 255)
            points = np.where(condition)[1]

            if points.any():
                centre = int(numpy.average(points))
                black_frame = cv.circle(black_frame, [int(centre), int((square_high + square_low) / 2)],
                                        radius=0, color=(0, 0, 255), thickness=15)
                centre_points.append([centre, int((square_high + square_low) / 2)])

            square_high += bar_height
            square_low += bar_height

        return frame, centre_points

    def create_binary_mask(self, frame):
        """
        :param frame: current frame
        :return: binary mask of frame made using self.low_green and self.high_green as the HSV range
        """

        # Run averaging filter to blur the frame
        kernel = np.ones((5, 5), np.float32) / 25
        frame = cv.filter2D(frame, -1, kernel)

        # Convert to hsv format to allow for easier colour filtering
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Filter image and allow only shades of green to pass
        mask = cv.inRange(hsv, self.LOW_GREEN, self.HIGH_GREEN)

        return mask
