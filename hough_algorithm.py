from cv2 import cv2
import numpy as np
import sys

import Lines


class hough_algorithm:

    def __init__(self):
        # Files, setup, OpenCV Tutorial
        # Master, demo
        # green filter
        self.LOWER_GREEN = np.array([25, 61, 70])
        self.UPPER_GREEN = np.array([55, 190, 177])

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
<<<<<<< HEAD
        self.resizeFactor = 1
             
=======
        self.resizeFactor = 2

>>>>>>> origin/RithinBranch
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

        # show the frames on screen for debugging
        # cv2.imshow('frame', frame)
        # cv2.imshow('mask', mask)
        # cv2.imshow('edges', edges)
        cv2.imshow('hough algorithm', lineimg)

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
<<<<<<< HEAD
        return cv2.resize(frame, (int(frame.shape[1]/factor), int(frame.shape[0]/factor)))

    # helper function to draw lines on given frame
    def drawp(self, lines, frame):

        if lines is not None:
            for x1,y1,x2,y2 in lines[:,0,:]:
                # Avoids math error, and we can skip since we don't care about horizontal lines
                if x1 == x2:
                    continue
                slope = (float(y2-y1))/(x2-x1)
                # Check if slope is sufficiently large, since we are interested in vertical lines
                if abs(slope)>1:
                    cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),4)
        return frame

    # helper function that finds an array of intersection points given an array of lines
    def intersectPoint(self, frame, lines):
        intersections = []
        points = []
        lines_left = []
        lines_right = []

        # check the lines array, only run if the lines array is not empty
        if lines is not None:

            # for each line in the lines array, we determine its slope.
            for line in lines:
                x1, y1, x2, y2 = line[0]

                # if the line is a vertical line, skip it
                if x2 == x1:
                    continue
                else:
                    slope = (y2-y1)/(x2-x1)
                
                # we only want to process lines that have a steep slope
                # keep in mind the x values increase to the right, y values increase downwards in an image
                if slope > 1 or slope < -1:
                    cv2.line(frame, (x1,y1), (x2,y2), (0,0,255), 1)
                    
                    # we split the lines into two different lists by their slopes (positive vs. negative)
                    # we call these two list lines_right and lines_left
                    if slope > 0:
                        lines_right.append(line)
                    else:
                        lines_left.append(line)

            # for each possible pairs of line that can be made from lines_right and lines_left, 
            # we find an intersection point
            for lineL in lines_left:
                for lineR in lines_right:
                    x1L, y1L, x2L, y2L = lineL[0]
                    x1R, y1R, x2R, y2R = lineR[0]
                    
                    # calls the getIntersection helper function
                    intersect = self.getIntersection(((x1L, y1L), (x2L, y2L)), ((x1R, y1R), (x2R, y2R)))
                    if type(intersect) is bool:
                        continue

                    # the intersections array is an array of points coordinates (x, y)
                    # the points array is an array of the x coordinates of the points
                    intersections.append(intersect)
                    points.append(intersect[0])
            
        return intersections, points
    
    
    # helper function to help determine the intersection point coordinate given two lines
    def getIntersection(self, line1, line2):

        # line has the following structure ((x1,y1), (x2,y2))
        s1 = np.array(line1[0])
        e1 = np.array(line1[1])

        s2 = np.array(line2[0])
        e2 = np.array(line2[1])

        # a1 is the slope of line1
        a1 = (s1[1] - e1[1]) / (s1[0] - e1[0])
        # b1 is the y-int of line1
        b1 = s1[1] - (a1 * s1[0])

        # a2 is the slope of line2
        a2 = (s2[1] - e2[1]) / (s2[0] - e2[0])
        # b2 is the y-int of line2
        b2 = s2[1] - (a2 * s2[0])

        # check if a1 and a2 are the same (epsilon is a very small value: 10^-16)
        if abs(a1 - a2) < sys.float_info.epsilon:
            return False

        # x coordinate is obtained by equating two mx+b equations where m is a1, a2 and b is b1, b2
        x = (b2 - b1) / (a1 - a2)
        # y coordinate is just obtained by plugging in one of the slope equations (since point must be on both lines)
        y = a1 * x + b1
        return (x, y)

    # helper function to draw the vanishing point on frame
    def vanishingPoint(self, frame, points):
        if len(points) is not 0:
            IntersectingX = np.average(points)
            cv2.circle(frame, (int(IntersectingX), int(frame.shape[1]/2)), 8, (255, 255, 255), -1)
            return (int(IntersectingX), int(frame.shape[1]/2))
=======
        return cv2.resize(frame, (int(frame.shape[1] / factor), int(frame.shape[0] / factor)))
>>>>>>> origin/RithinBranch
