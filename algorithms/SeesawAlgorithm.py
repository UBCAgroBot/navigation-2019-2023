import math

import cv2 as cv
import numpy as np
from algorithms.Algorithm import Algorithm

from algorithms.utils import Lines


class SeesawAlgorithm(Algorithm):

    def __init__(self, config):
        """
        Sets center row algorithm configurations
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
        self.HEIGHT = config.frame_length
        self.WIDTH = config.frame_width

    def process_frame(self, frame, show):
        bar_height = int(90)

        mask = self.create_binary_mask(frame)

        height = int(self.HEIGHT)
        width = int(self.WIDTH)
        half_width = int(self.WIDTH/2)

        left = mask[0:height, 0: half_width]
        right = mask[0: height, half_width + 1:width]

        cv.imshow("left", left)
        cv.imshow("right", right)
        cv.imshow("original", mask)
        #
        # # counting the number of pixels
        # white_left = np.sum(left == 255)
        # white_right = np.sum(right == 255)
        #
        # print('num of white in left:', white_left)
        # print('num of white in right:', white_right)

        square_low = 0
        square_high = bar_height
        left_array = []
        right_array = []
        points = []

        both_points = []

        black_frame = frame

        while square_low < self.HEIGHT:

            seg_left = left[int(square_low) + 1:int(square_high), 0:half_width]
            seg_right = right[int(square_low) + 1:int(square_high), 0:half_width]

            x1 = half_width - int(np.sum(seg_left == 255) / bar_height)
            x2 = half_width + int(np.sum(seg_right == 255) / bar_height)

            black_frame = cv.rectangle(black_frame, (half_width, square_low), (
                x1, int(square_high)), (255, 255, 255), 3)
            black_frame = cv.rectangle(black_frame, (half_width, square_low), (
                x2, int(square_high)), (255, 255, 255), 3)

            point_left = [int((x1 + half_width) / 2), int((square_high + square_low) / 2)]
            point_right = [int((x2 + half_width) / 2), int((square_high + square_low) / 2)]
            points.append(point_right)
            points.append(point_left)

            both_points.append([int((x1 + x2) / 2), int((square_high + square_low) / 2)])

            black_frame = cv.circle(black_frame, point_right, radius=0, color=(0, 0, 255), thickness=10)
            black_frame = cv.circle(black_frame, point_left, radius=0, color=(0, 0, 255), thickness=10)

            square_high += bar_height
            square_low += bar_height
            left_array.append(np.sum(seg_left == 255))
            right_array.append(np.sum(seg_right == 255))

        points = np.array(points)
        both_points = np.array(both_points)

        [vx, vy, x, y] = cv.fitLine(points, cv.DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x * vy / vx) + y)
        righty = int(((width - x) * vy / vx) + y)
        black_frame = cv.line(black_frame, (width - 1, righty), (0, lefty), (255, 255, 0), 9)

        [vx, vy, x, y] = cv.fitLine(both_points, cv.DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x * vy / vx) + y)
        righty = int(((width - x) * vy / vx) + y)
        black_frame = cv.line(black_frame, (width - 1, righty), (0, lefty), (0, 255, 0), 9)

        cv.imshow("test", black_frame)

        print(points)
        return mask, None

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
