from cv2 import cv2
import numpy as np
import sys

import Lines


class hough_algorithm:

    def __init__(self):
        # Files, setup, OpenCV Tutorial
        # Master, demo
        # green filter
        self.LOWER_GREEN = np.array([31, 43, 23])
        self.UPPER_GREEN = np.array([255, 255, 100])

        # gaussian blur / dilate
        self.K_SIZE = (3, 3)

        # canny edge
        self.CANNY_THRESH_1 = 600
        self.CANNY_THRESH_2 = 400

        # prob. hough
        self.THRESHOLD = 40
        self.MIN_LINE_LENGTH = 40
        self.MAX_LINE_GAP = 15

        # resize factor
        self.resizeFactor = 2

    # processFrame function that is called to process a frame of a video
    # takes in frame mat object obtained from cv2 video.read()
    def processFrame(self, frame):
        # create mask by filtering image colors
        mask = self.createMask(frame)

        # dilate the mask
        mask = self.dilate(mask)

        # Resize frame to smaller size to allow faster processing
        frame = self.resize(frame, self.resizeFactor)
        mask = self.resize(mask, self.resizeFactor)

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

        return lineimg, vPoint
        # show the frames on screen for debugging
        # cv2.imshow('frame', frame)
        # cv2.imshow('mask', mask)
        # cv2.imshow('edges', edges)
        # cv2.imshow('hough algorithm', lineimg)

    # helper function to create a mask
    # takes in frame mat object, returns mask mat object
    def createMask(self, frame):
        # Convert to hsv format to allow for easier colour filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Filter image and allow only shades of green to pass
        mask = cv2.inRange(hsv, self.LOWER_GREEN, self.UPPER_GREEN)
        # Apply gaussian blur (can be removed)
        mask = cv2.GaussianBlur(mask, self.K_SIZE, 2)

        return mask

    # helper function to dilate a mask mat object
    def dilate(self, mask):
        # Perform dilation on mask
        # Dilation helps fill in gaps within a single row
        # In addition it helps blend rows that are far from the camera together
        # Hence, we get cleaner edges when we perform Canny edge detection
        kernel = np.ones(self.K_SIZE, np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=4)

        return mask

    # helper function to resize a frame mat object
    def resize(self, frame, factor):
        # resize frame to smaller size to allow faster processing
        return cv2.resize(frame, (int(frame.shape[1] / factor), int(frame.shape[0] / factor)))
