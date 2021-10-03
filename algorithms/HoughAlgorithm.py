from cv2 import cv2
import numpy as np
import sys
from algorithms.utils import Lines


class HoughAlgorithm:

    def __init__(self, config):
        # Files, setup, OpenCV Tutorial
        # Master, demo
        self.config = config

        self.LOWER_GREEN = np.array(config.lower_hsv_threshold_hough)
        self.UPPER_GREEN = np.array(config.upper_hsv_threshold_hough)


        # gaussian blur / dilate
        self.K_SIZE = (config.blur_size_1, config.blur_size_2)

        # canny edge
        self.CANNY_THRESH_1 = config.canny_thresh_1
        self.CANNY_THRESH_2 = config.canny_thresh_2

        # prob. hough
        self.THRESHOLD = config.hough_threshold
        self.MIN_LINE_LENGTH = config.hough_min_line_length
        self.MAX_LINE_GAP = config.hough_max_line_gap

        # resize factor
        self.RESIZE_FACTOR = config.resize_factor

    # processFrame function that is called to process a frame of a video
    # takes in frame mat object obtained from cv2 video.read()
    def processFrame(self, frame, show=True):
        # create mask by filtering image colors
        mask = self.createMask(frame)

        # Resize frame to smaller size to allow faster processing
        frame = self.resize(frame, self.RESIZE_FACTOR)
        # mask = self.resize(mask, self.RESIZE_FACTOR)
    
        # Perform Canny Edge Detection
        edges = cv2.Canny(mask, self.CANNY_THRESH_1, self.CANNY_THRESH_2)

        # Perform Hough Lines Probabilistic Transform
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180,
            self.THRESHOLD,
            np.array([]),
            self.MIN_LINE_LENGTH,
            maxLineGap=self.MAX_LINE_GAP
        )

        # Draw Detected Lines on the frame
        lineimg = Lines.drawLinesOnFrame(lines, frame.copy())

        # Draw the vanishing point obtained fromm all the lines
        # intersections, points = self.intersectPoint( frame, lines)
        intersections, points = Lines.getIntersections(lines)
        vPoint = Lines.drawVanishingPoint(lineimg, points)

        
        # show the frames on screen for debugging
        if show:
            cv2.imshow('frame', frame)
            cv2.imshow('mask_hough',mask)
            cv2.imshow('edges', edges)
            cv2.imshow('hough algorithm', lineimg)
            cv2.waitKey(1)

        return lineimg, vPoint

    # helper function to create a mask
    # takes in frame mat object, returns mask mat object
    def createMask(self, frame):
        # Convert to hsv format to allow for easier colour filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Filter image and allow only shades of green to pass
        mask = cv2.inRange(hsv, self.LOWER_GREEN, self.UPPER_GREEN)
        # Apply gaussian blur (can be removed)
        mask = cv2.GaussianBlur(mask, self.K_SIZE, 2)
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
        mask = cv2.dilate(mask, kernel, iterations=1)

        return mask

    # helper function to resize a frame mat object
    def resize(self, frame, factor):
        # resize frame to smaller size to allow faster processing
        return cv2.resize(frame, (int(frame.shape[1] / factor), int(frame.shape[0] / factor)))
