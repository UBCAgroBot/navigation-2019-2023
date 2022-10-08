import cv2 as cv2
import numpy as np
import sys
import math
from algorithms.utils import Lines


class HoughAlgorithm:

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

    # processFrame function that is called to process a frame of a video
    # takes in frame mat object obtained from cv2 video.read()
    def processFrame(self, frame, show=True):
        #pts = [(0, 400), (400,800), (0, 800)]
        #cv2.fillPoly(frame, np.array([pts]), (0,0,0))
        #pts = [(800, 400), (400,800), (800, 800)]
        #cv2.fillPoly(frame, np.array([pts]), (0,0,0))


        # create mask by filtering image colors
        mask = self.createMask(frame)

        # Resize frame to smaller size to allow faster processing
        frame = self.resize(frame, self.RESIZE_FACTOR)
        # mask = self.resize(mask, self.RESIZE_FACTOR)
    
        # Perform Canny Edge Detection
        v = np.median(mask)
        self.CANNY_THRESH_1 = int(max(0, (1.0 - self.CANNY_SIGMA) * v))
        self.CANNY_THRESH_2 = int(min(255, (1.0 + self.CANNY_SIGMA) * v))

        edges = cv2.Canny(mask, self.CANNY_THRESH_1, self.CANNY_THRESH_2)


        # Perform Hough Lines Probabilistic Transform
        lines = cv2.HoughLinesP(
            edges,
            self.HOUGH_RHO,
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
        intersections = Lines.getIntersections(lines)
        xPoints = [point[0] for point in intersections]
        yPoints = [point[1] for point in intersections]
        vPoint = Lines.drawVanishingPoint(lineimg, xPoints, yPoints)

        # Calculating angle from vanishing point to (self.WIDTH // 2, 0)
        deltaWVanishPoint = vPoint[0] - (self.WIDTH // 2)
        deltaHVanishPoint = vPoint[1]
        angle = round(math.degrees(math.atan(deltaWVanishPoint/deltaHVanishPoint)), 2)
        
        # show the frames on screen for debugging
        # if show:
        #     cv2.imshow('frame', frame)
        #     cv2.imshow('mask_hough',mask)
        #     cv2.imshow('edges', edges)
        #     cv2.imshow('hough algorithm', lineimg)
        #     cv2.waitKey(1)

        return lineimg, angle

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
