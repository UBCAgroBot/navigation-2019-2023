import cv2 as cv
import numpy as np
import sys
import math
from algorithms.utils import Lines
from algorithms.Algorithm import Algorithm


class HoughAlgorithm(Algorithm):

    def __init__(self, config):
        # Files, setup, OpenCV Tutorial
        # Master, demo
        self.config = config
        self.WIDTH = config.frame_width
        self.HEIGHT = config.frame_length

        self.LOWER_GREEN = np.array(config.lower_hsv_threshold_hough)
        self.UPPER_GREEN = np.array(config.upper_hsv_threshold_hough)

        self.CANNY_SIGMA = config.canny_sigma

        # gaussian blur / dilate
        self.K_SIZE = (config.blur_size_1, config.blur_size_2)

        # prob. hough
        self.THRESHOLD = config.hough_threshold
        self.MIN_LINE_LENGTH = config.hough_min_line_length
        self.MAX_LINE_GAP = config.hough_max_line_gap
        self.HOUGH_RHO = config.hough_rho

        # resize factor
        self.RESIZE_FACTOR = config.resize_factor

    def get_extra_content(self, frame, show):
        item1, item2 = self.process_frame(frame, show)
        return item1, item2

    def update_lower_hsv(self, next):
        self.LOW_GREEN = np.array(next)

    def update_upper_hsv(self, next):
        self.HIGH_GREEN = np.array(next)

    # processFrame function that is called to process a frame of a video
    # takes in frame mat object obtained from cv2 video.read()
    def process_frame(self, frame, show=True):
        # pts = [(0, 400), (400,800), (0, 800)]
        # cv2.fillPoly(frame, np.array([pts]), (0,0,0))
        # pts = [(800, 400), (400,800), (800, 800)]
        # cv2.fillPoly(frame, np.array([pts]), (0,0,0))

        # create mask by filtering image colors
        mask = self.create_mask(frame)

        # Resize frame to smaller size to allow faster processing
        frame = self.resize(frame, self.RESIZE_FACTOR)
        # mask = self.resize(mask, self.RESIZE_FACTOR)

        # Perform Canny Edge Detection
        v = np.median(mask)
        self.CANNY_THRESH_1 = int(max(0, (1.0 - self.CANNY_SIGMA) * v))
        self.CANNY_THRESH_2 = int(min(255, (1.0 + self.CANNY_SIGMA) * v))

        edges = cv.Canny(mask, self.CANNY_THRESH_1, self.CANNY_THRESH_2)

        # Perform Hough Lines Probabilistic Transform
        lines = cv.HoughLinesP(
            edges,
            self.HOUGH_RHO,
            np.pi / 180,
            self.THRESHOLD,
            np.array([]),
            self.MIN_LINE_LENGTH,
            maxLineGap=self.MAX_LINE_GAP
        )

        if show:
            line_img = Lines.draw_lines_on_frame(lines, frame.copy())

        # Draw the vanishing point obtained fromm all the lines
        # intersections, points = self.intersectPoint( frame, lines)
        intersections = Lines.get_intersections(lines)

        if not intersections:
            if show:
                return line_img, None
            else:
                return frame, None

        x_points = [point[0] for point in intersections]
        y_points = [point[1] for point in intersections]

        if show:
            vanishing_point = Lines.draw_vanishing_point(
                line_img, x_points, y_points, show)
        else:
            vanishing_point = Lines.draw_vanishing_point(
                frame, x_points, y_points, show)

        # Calculating angle from vanishing point to (self.WIDTH // 2, 0)
        angle = Lines.calculate_angle_from_v_point(
            vanishing_point, self.WIDTH, self.HEIGHT)

        if show:
            return line_img, angle
        else:
            return frame, angle

    # helper function to create a mask
    # takes in frame mat object, returns mask mat object
    def create_mask(self, frame):
        # Convert to hsv format to allow for easier colour filtering
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # Filter image and allow only shades of green to pass
        mask = cv.inRange(hsv, self.LOWER_GREEN, self.UPPER_GREEN)
        # Apply gaussian blur (can be removed)
        mask = cv.GaussianBlur(mask, self.K_SIZE, 2)
        # dilate the mask
        mask = self.dilate(mask)

        return mask

    # helper function to dilate a mask mat object
    def dilate(self, mask):
        # Perform dilation on mask
        # Dilation helps fill in gaps within a single row
        # In addition it helps blend rows that are far from the camera together
        # Hence, we get cleaner edges when we perform Canny edge detection
        kernel = np.ones(self.K_SIZE, np.uint8)
        mask = cv.dilate(mask, kernel, iterations=1)

        return mask

    # helper function to resize a frame mat object
    def resize(self, frame, factor):
        # resize frame to smaller size to allow faster processing
        return cv.resize(frame, (int(frame.shape[1] / factor), int(frame.shape[0] / factor)))
