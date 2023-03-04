import cv2
import numpy as np
import time
import operator
import sys
import math
from algorithms.utils import Lines


class MiniContoursDownwards():

    def __init__(self, config):

        # width and height of frame
        self.config = config
        self.WIDTH = config.frame_width
        self.HEIGHT = config.frame_length
        # smoothing kernel
        self.kernel = np.ones((5, 5), np.float32) / 25
        self.morphology_kernel = np.ones((9, 9), np.float32)

        # thresholds for the color of the crop rows
        self.low_green = np.array(config.low_green)
        self.high_green = np.array(config.high_green)

        # random colors for drawing lines
        # teal (for contour points that are within bounds)
        self.color_1 = (255, 255, 0)
        # pink (for the line of best fit through all the contour points that are within bounds)
        self.color_2 = (200, 0, 255)
        # red (for contour points that are out of bounds)
        self.color_3 = (0, 0, 255)
        # orange (for a reference line vertically down center of frame)
        self.midline = (0, 129, 255)

        # parameters for calculating centroids and drawing the best fit line among them
        self.num_strips = self.config.num_strips
        self.param = self.config.param
        self.reps = self.config.reps
        self.aeps = self.config.aeps

    def apply_filters(self, original_frame):
        """""
        parameters:
        - orginal_frame: BGR frame
        returns:
        - frame: filtered BGR frame
        """""

        frame = cv2.filter2D(original_frame, -1, self.kernel)
        frame = cv2.bilateralFilter(frame, 9, 10, 75)

        return frame

    def get_centroids(self, mask, num_strips, show):
        """""
        parameters:
        - mask is a binary mask of the image
        - num_strips is the number of strips to divide the image into for finding centroids
        returns:
        - list [(x1, y1), (x2, y2), ...] of all the centroids obtained
        """""

        strips = []
        width = int(mask.shape[0] / num_strips)
        for i in range(100):
            strips.append(mask[i * width:(i + 1) * width, 0:mask.shape[1]])

        centroids = []
        for i, strip in enumerate(strips):
            contours, hierarchy = cv2.findContours(
                strip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            strip_centroids = []
            for contour in contours:
                M = cv2.moments(contour)
                try:
                    cX = int(M["m10"] / M["m00"])
                    strip_centroids.append((cX, strip.shape[0] * (i + 0.5)))
                except BaseException:
                    pass
            centroids.append(strip_centroids)

        return centroids

    def get_best_fit_line(self, frame, show, num_strips=60, dist_type=cv2.DIST_L2, param=0, reps=0.01, aeps=0.01):
        """""
        parameters:
        - frame: BGR frame
        - num_strips: number of strips for centroid calculation
        - other parameters required to calculate line of best fit through centroids using cv2.fitLine
        returns:
         -frame: original frame with best fit line drawn on
         -line: A vector [vx, vy, x, y] representing the line of best fit through all the centroids. [vx, vy] is a vector that describes the direction of the line, where (x, y) when taken together is a point on the line
        """""

        mask = cv2.inRange(cv2.cvtColor(
            frame, cv2.COLOR_BGR2HSV), self.low_green, self.high_green)
        mask = cv2.medianBlur(mask, 9)
        # mask = cv2.GaussianBlur(mask, (9,9), 10)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morphology_kernel)
        centroids = self.get_centroids(mask, num_strips=num_strips, show=show)
        points = np.zeros(mask.shape, dtype=np.uint8)
        # points = cv2.Mat.zeros(mask.shape[0], mask.shape[1], cv.CV_8UC3)
        points_vector = []

        height, width = frame.shape[0], frame.shape[1]

        split_factor = 10
        segmented_points = [[] for _ in range(split_factor)]

        for i, strip_centroid in enumerate(centroids):
            for centroid in strip_centroid:
                x, y = centroid[0], centroid[1]

                # vertically split the points
                idx = int(x / width * split_factor)
                segmented_points[idx].append([int(x), int(y)])
                if show:
                    cv2.circle(frame, (int(centroid[0]), int(
                        centroid[1])), 3, self.color_1, -1)
                    cv2.circle(mask, (int(centroid[0]), int(
                        centroid[1])), 3, self.color_1, -1)
                points_vector.append([int(centroid[0]), int(centroid[1])])

        c_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        for point in points_vector:
            try:
                points[point[1]][point[0]] = 255
            except BaseException:
                pass

        # if there are less than 5 points in the vector, we don't draw the line
        # since it will give a poor approximation of the actual crop row
        if len(points_vector) > 5:
            points_vector = np.array([points_vector])
            line = cv2.fitLine(points_vector,
                               distType=dist_type,
                               param=param,
                               reps=reps,
                               aeps=aeps
                               )
            # want to show the line
            if show:
                cv2.line(frame,
                         (int(line[2]) - self.WIDTH * int(1000 * line[0]),
                          int(line[3]) - self.HEIGHT * int(1000 * line[1])),
                         (int(line[2]) + self.WIDTH * int(1000 * line[0]),
                             int(line[3]) + self.HEIGHT * int(1000 * line[1])),
                         self.color_2,
                         3)
        else:
            line = None

        if show:
            cv2.imshow('frame', frame)
            cv2.imshow('mask', mask)
            cv2.imshow('c_mask', c_mask)
            cv2.imshow('points', points)

        return frame, line

    def get_extra_content(self, frame, show):
        item1, item2 = self.process_frame(frame, show)
        return item1, item2

    def update_lower_hsv(self, next):
        self.LOW_GREEN = np.array(next)

    def update_upper_hsv(self, next):
        self.HIGH_GREEN = np.array(next)

    def process_frame(self, original_frame, num_strips=60, show=False):
        """""
        parameters:
        - original_frame: BGR frame
        - num_strips: set in config, the number of strips used when calculating centroids
        returns:
        - frame: original_frame with the best fit line and centroids drawn on
        - angle: angle between the best fit line and a line drawn vertically down the center of the screen
        """""

        frame = self.apply_filters(original_frame)

        frame, line = self.get_best_fit_line(frame,
                                             show,
                                             num_strips=num_strips,
                                             dist_type=cv2.DIST_L2,
                                             param=self.param,
                                             reps=self.reps,
                                             aeps=self.aeps)

        if line[0] is not None:
            angle = round(math.degrees(math.atan(-line[0] / line[1])), 2)
            cv2.line(frame, (int(self.WIDTH / 2), 0),
                     (int(self.WIDTH / 2), int(self.HEIGHT)), self.midline, 2)
            print(angle)
            return frame, angle
        else:
            angle = None
            return frame, angle
