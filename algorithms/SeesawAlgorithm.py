import math

import cv2 as cv
import numpy as np
from algorithms.Algorithm import Algorithm


class SeesawAlgorithm(Algorithm):

    def __init__(self, config):

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

    def process_frame(self, frame, show):

        black_frame, points, both_points = self.plot_points(frame)
        points = np.array(points)
        both_points = np.array(both_points)

        # """get best fit line for left points and right points"""
        # [vx, vy, x, y] = cv.fitLine(points, cv.DIST_L2, 0, 0.01, 0.01)
        # black_frame = cv.line(black_frame, (int(x - vx * self.WIDTH), int(y - vy * self.HEIGHT)),
        #                       (int(x + vx * self.WIDTH), int(y + vy * self.HEIGHT)), (0, 0, 255), 9)

        """get best fit line for middle points"""
        [vx, vy, x, y] = cv.fitLine(both_points, cv.DIST_L2, 0, 0.01, 0.01)
        x1 = int(x - vx * self.WIDTH)
        x2 = int(x + vx * self.WIDTH)
        y1 = int(y - vy * self.HEIGHT)
        y2 = int(y + vy * self.HEIGHT)
        black_frame = cv.line(black_frame, (x1, y1), (x2, y2), (0, 255, 255), 9)

        """calculate angle"""
        if y1 - y2 != 0:
            angle = round(math.degrees(math.atan(int(x2 - x1) / int(y1 - y2))), 2)
        else:
            angle = None

        return black_frame, angle

    def plot_points(self, frame):
        """This value needs to be changed to change the height of the bars"""
        bar_height = int(90)

        mask = self.create_binary_mask(frame)
        half_width = int(self.WIDTH / 2)

        left = mask[0: self.HEIGHT, 0: half_width]
        right = mask[0: self.HEIGHT, half_width + 1: self.WIDTH]

        square_low = 0
        square_high = bar_height
        both_points = []

        xs = []

        normalized = False

        black_frame = frame

        """draw rectangle and point for every square, add points to array"""
        while square_low < self.HEIGHT:
            normalized = False
            seg_left = left[int(square_low) + 1:int(square_high), 0:half_width]
            seg_right = right[int(square_low) + 1:int(square_high), 0:half_width]

            left_x = int(np.sum(seg_left == 255) / bar_height)
            right_x = int(np.sum(seg_right == 255) / bar_height)

            if left_x > half_width/2 or right_x > half_width/2:
                normalized = True

            xs.append([left_x, right_x])

            x1 = half_width - left_x
            x2 = half_width + right_x

            black_frame = cv.rectangle(black_frame, (half_width, square_low), (
                x1, int(square_high)), (255, 255, 0), 3)
            black_frame = cv.rectangle(black_frame, (half_width, square_low), (
                x2, int(square_high)), (255, 255, 0), 3)

            both_point = [int((x1 + x2) / 2), int((square_high + square_low) / 2)]
            both_points.append(both_point)

            black_frame = cv.circle(black_frame, both_point, radius=0, color=(0, 255, 255), thickness=10)

            square_high += bar_height
            square_low += bar_height

        if normalized is False:
            for points in xs:
                points[0] *= 2
                points[1] *= 2

        square_low = 0
        square_high = bar_height

        for points in xs:
            x1 = half_width - points[0]
            x2 = half_width + points[1]

            black_frame = cv.rectangle(black_frame, (half_width, square_low), (
                x1, int(square_high)), (255, 255, 255), 3)
            black_frame = cv.rectangle(black_frame, (half_width, square_low), (
                x2, int(square_high)), (255, 255, 255), 3)

            square_high += bar_height
            square_low += bar_height

        return black_frame, points, both_points

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
