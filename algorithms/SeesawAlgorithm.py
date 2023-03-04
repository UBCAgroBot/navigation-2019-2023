import math

import cv2 as cv
import numpy
import numpy as np
from algorithms.Algorithm import Algorithm
from helper_scripts.change_res import change_res


class SeesawAlgorithm(Algorithm):

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
        self.NORM_FACTOR = config.normalization_factor
        self.RES_FACTOR = config.resolution_factor

        # output adjustment
        self.k = config.k

    def process_frame(self, frame, show):
        """
        Divides screen into horizontal strips and draw bars according to the amount
        of green present on each strip. Draw the best fit line based on the centre of each bar,
        then calculate the angle between the best fit line and a horizontal line.
        :param frame: current frame (mat)
        :param show: show/hide frames on screen for debugging
        :type show: bool
        :return: processed frame (mat), angle [-90, 90]
        """
        frame = change_res(frame, self.RES_FACTOR)
        # black_frame, both_points = self.plot_points(frame)
        black_frame, both_points = self.plot_points_2(frame)
        both_points = np.array(both_points)

        if both_points.any():
            """get best fit line for centre points"""
            [vx, vy, x, y] = cv.fitLine(both_points, cv.DIST_L2, 0, 0.01, 0.01)
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

        # alternative way of calculating angle

        # angle = 0
        # for point in both_points:
        #     if point[0] > self.WIDTH / 2:
        #         angle += 1
        #     elif point[0] < self.WIDTH / 2:
        #         angle -= 1
        #
        # if abs(angle) >= self.k:
        #     angle = round(angle/(len(both_points))*90, 1)
        # else:
        #     angle = 0

        return black_frame, angle

    def plot_points_2(self, frame):

        bar_height = int(self.BAR_HEIGHT)
        mask = self.create_binary_mask(frame)

        square_low = 0
        square_high = bar_height

        black_frame = frame

        centre_points = []

        while square_low < self.HEIGHT:
            points = []

            seg = mask[int(square_low) + 1:int(square_high), 0:self.WIDTH]

            # iterable = (index for index, x in enumerate(seg[0]) if x == 255)
            # points = np.fromiter(iterable, int)

            for iy, ix in np.ndindex(seg.shape):
                if seg[iy, ix] == 255:
                    points.append(ix)

            # for strip in seg:
            #     iterable = (index for index, x in enumerate(strip) if x == 255)
            #     points = np.fromiter(iterable, int)

            if points:
                centre = int(numpy.average(points))
                black_frame = cv.circle(black_frame, [int(centre), int((square_high + square_low) / 2)],
                                        radius=0, color=(0, 0, 255), thickness=15)
                centre_points.append([centre, int((square_high + square_low) / 2)])

            square_high += bar_height
            square_low += bar_height

        return frame, centre_points

    def plot_points(self, frame):
        """
        Divides screen into equally sized rectangles on the left and right side
        and draw bars according to the amount of green present on each strip.
        Calculates the centre point of each horizontal strip.
        :param frame: current frame (mat)
        :return: processed frame (mat), list of centre points
        """

        # initializing parameters
        bar_height = int(self.BAR_HEIGHT)
        mask = self.create_binary_mask(frame)
        half_width = int(self.WIDTH / 2)

        left = mask[0: self.HEIGHT, 0: half_width]
        right = mask[0: self.HEIGHT, half_width + 1: self.WIDTH]

        square_low = 0
        square_high = bar_height
        both_points = []
        xs = []

        normalized = False
        isnull = True

        black_frame = frame

        while square_low < self.HEIGHT:

            # for each area, calculates the amount of green present
            normalized = False

            seg_left = left[int(square_low) + 1:int(square_high), 0:half_width]
            seg_right = right[int(square_low) + 1:int(square_high), 0:half_width]

            left_x = int(np.sum(seg_left == 255) / bar_height)
            right_x = int(np.sum(seg_right == 255) / bar_height)

            print("lol", seg_left)

            if left_x > half_width / 2 or right_x > half_width / 2:
                normalized = True

            if left_x != 0 or right_x != 0:
                isnull = False

            xs.append([left_x, right_x])

            # draw bars based on the amount of green present
            x1 = half_width - left_x
            x2 = half_width + right_x

            black_frame = cv.rectangle(black_frame, (half_width, square_low),
                                       (x1, int(square_high)), (255, 255, 0), 3)
            black_frame = cv.rectangle(black_frame, (half_width, square_low),
                                       (x2, int(square_high)), (255, 255, 0), 3)

            # draw centre points of the bars, add points to a list
            both_point = [int((x1 + x2) / 2), int((square_high + square_low) / 2)]
            both_points.append(both_point)

            black_frame = cv.circle(black_frame, both_point, radius=0, color=(0, 0, 255), thickness=15)

            square_high += bar_height
            square_low += bar_height

        # normalize the lengths of the bars
        if isnull:
            normalized = True

        while normalized is False:
            for points in xs:
                points[0] = float(self.NORM_FACTOR * points[0])
                points[1] = float(self.NORM_FACTOR * points[1])

                if points[0] > half_width / 2 or points[1] > half_width / 2:
                    normalized = True

        # draw normalized bars
        square_low = 0
        square_high = bar_height

        for points in xs:
            x1 = int(half_width - points[0])
            x2 = int(half_width + points[1])

            black_frame = cv.rectangle(black_frame, (half_width, square_low), (
                x1, int(square_high)), (255, 255, 255), 3)
            black_frame = cv.rectangle(black_frame, (half_width, square_low), (
                x2, int(square_high)), (255, 255, 255), 3)

            square_high += bar_height
            square_low += bar_height

        return black_frame, both_points

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
