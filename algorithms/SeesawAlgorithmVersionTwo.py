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
        black_frame, average_points, overall_bias = self.plot_points(frame)
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
                bias = round(numpy.average(overall_bias) / (self.WIDTH / 2) * 90, 2)

                if abs(angle) > abs(bias):
                    output = angle
                else:
                    output = bias
            else:
                output = None
        else:
            output = None

        black_frame = self.print_text(frame, str(output))

        return black_frame, output

    def plot_points(self, frame):
        """
        Divides screen into horizontal strips, and find average location of green for each strip.
        :param frame: current frame (mat)
        :return: processed frame (mat), list of centre points
        """
        frame = cv.line(frame, (int(self.WIDTH / 2), 0), (int(self.WIDTH / 2), int(self.HEIGHT)), (255, 0, 255), 9)
        bar_height = int(self.BAR_HEIGHT)
        mask = self.create_binary_mask(frame)

        square_low = 0
        square_high = bar_height

        black_frame = frame

        centre_points = []
        overall_bias = []

        while square_low < self.HEIGHT:
            points = []

            seg = mask[int(square_low) + 1:int(square_high), 0:self.WIDTH]

            condition = (seg == 255)
            points = np.where(condition)[1]

            if points.any():
                centre = int(numpy.median(points))
                bias = int(centre - self.WIDTH / 2)
                black_frame = cv.circle(black_frame, [int(centre), int((square_high + square_low) / 2)],
                                        radius=0, color=(0, 0, 255), thickness=15)
                centre_points.append([centre, int((square_high + square_low) / 2)])
                overall_bias.append(bias)

            square_high += bar_height
            square_low += bar_height

        return frame, centre_points, overall_bias

    def print_text(self, frame, text):

        # Define the font and text size
        font = cv.FONT_HERSHEY_SIMPLEX
        font_scale = 1

        # Define the color and thickness of the text
        color = (255, 255, 255)  # in BGR format
        thickness = 2

        # Get the size of the text box
        text_size, _ = cv.getTextSize(text, font, font_scale, thickness)

        # Calculate the position of the text box
        x = int((frame.shape[1] - text_size[0]) / 4)
        y = int((frame.shape[0] + text_size[1]) / 4)

        # Draw the text on the image
        cv.putText(frame, text, (x, y), font, font_scale, color, thickness)

        return frame

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
