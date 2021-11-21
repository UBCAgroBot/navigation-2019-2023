import math

import cv2 as cv
import numpy as np

# from helper_scripts import hsv_threshold
from algorithms.utils import Lines


class CenterRowAlgorithm:

    # constructor -- assigns the following parameters
    def __init__(self, config):

        # hsv mask
        self.low_green = np.array(config.lower_hsv_threshold)
        self.high_green = np.array(config.upper_hsv_threshold)

        # filtering parameters
        self.averaging_kernel_size = config.averaging_kernel_size
        self.gauss_kernel_size = (3, 3)
        self.dilate_kernel_size = config.dilate_kernel_size
        self.sigmaX = config.sigmaX

        # contour parameters
        self.contour_color = (0, 129, 255)
        self.min_contour_area = config.min_contour_area
        self.max_contour_area = config.max_contour_area

        # Canny edge parameters
        self.canny_low_threshold = config.canny_low_threshold
        self.canny_high_threshold = config.canny_high_threshold

        # dimensions
        self.HEIGHT = config.frame_length
        self.WIDTH = config.frame_width

        # contours
        self.contours = []

    def processFrame(self, frame):
        # This function uses contouring to create contours around each crop row
        # and uses these contours to find centroid lines, and the correspond row vanishing point
        # This function takes in the current frame (mat) as a parameter, and returns the
        # vanishing point (tuple)

        black_frame = np.uint8(np.zeros((720, 1280, 3)))

        # h, w, c = frame.shape
        # frame = frame[h // 2:h, 0:w]

        mask = self.create_binary_mask(frame)

        # Perform canny edge detection
        edges = cv2.Canny(mask, self.cannyLowThld, self.cannyHighThld)
        blurred_edges = self.gaussian_blur(edges)

        cnt, contour_frame = self.get_contours(mask)
        cv2.drawContours(black_frame, cnt, -1, self.contourColor, 3)
        # fillPoly fills in the polygons in the frame
        cv2.fillPoly(black_frame, pts=cnt, color=self.contourColor)
        lines, slopes, ellipse_frame = self.ellipse_slopes(cnt, black_frame)
        Lines.drawLinesOnFrame(lines, black_frame)
        intersections, points = Lines.getIntersections(lines)
        vanishing_point = Lines.drawVanishingPoint(ellipse_frame, points)

        # print("vanishing point", vanishing_point)
        if vanishing_point:
            min_contour, angle = self.get_min_angle(vanishing_point, black_frame)
            cv.ellipse(black_frame, min_contour, (0, 255, 0), 2)

        return black_frame, vanishing_point

    def get_min_angle(self, vanishing_point, frame):

        min_del_x = math.inf
        min_angle = math.inf
        min_contour = None

        for contour in self.contours:
            center = contour[0]
            del_x = vanishing_point[0] - center[0]
            if abs(del_x) < min_del_x:
                del_y = vanishing_point[1] - 400 - center[1]
                angle = math.atan(del_x / del_y)
                min_angle = angle
                min_contour = contour
                min_del_x = abs(del_x)

        return min_contour, min_angle

    def create_binary_mask(self, frame):
        # Current frame is input as a parameter
        # The function uses HSV filtering with the specificed low_green to high_green HSV range
        # to binarize the image and returns the binary frame

        # Run averaging filter to blur the frame
        kernel = np.ones((5, 5), np.float32) / 25
        frame = cv2.filter2D(frame, -1, kernel)

        # Convert to hsv format to allow for easier colour filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Filter image and allow only shades of green to pass
        mask = cv2.inRange(hsv, self.low_green, self.high_green)

        return mask

    def gaussian_blur(self, frame):
        # Current frame is input as a parameter
        # This function applies gaussian blurring to the frame
        # And returns the resulting blurred frame
        mask = cv2.GaussianBlur(frame, self.gaussKernelSize, self.sigmaX)
        return mask

    def ellipse_slopes(self, cnt, black_frame):
        # Takes in the list of contours on the frame and the frame with contours (black frame) as parameters
        # This function draws ellipses around each contour on black_frame using the fitEllipse function
        # Uses the fitline function with the list of contours to create a set of lines
        # Returns an array of lines, an array of slopes and an updated frame with ellipses and lines
        slopes = []
        lines = []
        for cnt in cnt:
            if cv2.contourArea(cnt) > self.minContourArea:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                black_frame = cv.drawContours(black_frame, [box], 0, (255, 255, 255), 2)
                ellipse = cv.fitEllipse(cnt)
                # print("center", ellipse[0])
                self.contours.append(ellipse)
                cv.ellipse(black_frame, ellipse, (255, 255, 255), 2)
                rows, cols = black_frame.shape[:2]
                # Defines a line for the contour using the fitLine function
                [vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
                # calculates the slope and adds the slope to the array of slopes
                slope = vy / vx
                slopes.append(slope)
                # Finds two other points on the line using the slope
                lefty = int((-x * vy / vx) + y)
                righty = int(((cols - x) * vy / vx) + y)
                # black_frame = cv2.line(black_frame, (cols - 1, righty), (0, lefty), (255, 255, 0), 9)
                # Appends a line to the lines array using the (x1,y1,x2,y2) definition
                lines.append([cols - 1, righty, 0, lefty])

        return lines, slopes, black_frame

    def get_contours(self, binary_mask):
        # Takes in a binary image as a parameter
        # Uses the cv2 findContours function to find the contours (creates closed shapes around connected pixels) in the image
        # returns a list of contours and blank frame with contours filled in
        frame = np.zeros((self.HEIGHT, self.WIDTH, 3))
        ret, thresh = cv2.threshold(binary_mask, 0, 254, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours
        cv2.drawContours(frame, cnt, -1, (0, 255, 0), 2)
        cv2.fillPoly(frame, pts=cnt, color=(0, 255, 0))

        return cnt, frame

    
