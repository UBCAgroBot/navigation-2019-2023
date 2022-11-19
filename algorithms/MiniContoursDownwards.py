import math

import cv2 as cv
import numpy
import numpy as np
from algorithms.Algorithm import Algorithm
from helper_scripts.change_res import change_res
from algorithms.utils import Lines

class MiniContoursDownwards(Algorithm):

    def __init__(self, config):
        """
        Sets center row algorithm configurations
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

        # contour parameters
        self.contour_color = list(map(int, config.contour_color.split(',')))
        self.min_contour_area = config.min_contour_area

        # Canny edge parameters
        self.canny_low_threshold = config.canny_low_threshold
        self.canny_high_threshold = config.canny_high_threshold

        # dimensions
        self.HEIGHT = config.frame_length
        self.WIDTH = config.frame_width

        # contours
        self.contours = []

        # center contour
        self.center = None
        self.center_angle = 0

    def process_frame(self, frame, show):
        """Uses contouring to create contours around each crop row and uses these contours to find centroid lines,
        row vanishing point, a center contour and the angle between the center contour and vanishing point\n
        :param frame: current frame (mat)
        :param show: show/hide frames on screen for debugging
        :type show: bool
        :return: processed frame (mat), angle [-90, 90]
        """

        cv.imshow("original", frame)

        # frame = change_res(frame, 5)
        # cv.imshow("res", frame)

        mask = self.create_binary_mask(frame)
        cv.imshow("binary", mask)

        # gauss_mask = self.gaussian_blur(mask)
        # cv.imshow("blurred", gauss_mask)
        #
        # med_mask = cv.medianBlur(mask, 5)
        # cv.imshow("medblur", med_mask)

        # blur_mask = cv.blur(mask, (15, 15))
        # cv.imshow("blur", blur_mask)

        mask = self.morpholoical_op(mask, 9, 2, 1)
        cv.imshow("lol", mask)

        # mask = cv.bitwise_not(mask)
        #
        # kernel = np.array([[1, 1, 1],
        #                    [1, -2, 1],
        #                    [1, 1, 1]])
        #
        # mask = cv.filter2D(src=mask, ddepth=-1, kernel=kernel)
        #
        # mask = cv.bitwise_not(mask)
        # cv.imshow("lol", mask)

        # Perform canny edge detection
        # edges = cv.Canny(mask, self.canny_low_threshold, self.canny_high_threshold)
        # blurred_edges = self.gaussian_blur(edges)

        cont, cont_frame = self.get_contours(mask)

        for cnt in cont:
            if 5000 > cv.contourArea(cnt):
                cv.drawContours(mask, [cnt], 0, self.contour_color, 2)
                cv.fillPoly(mask, pts=[cnt], color=self.contour_color)

        cv.imshow("fillgaps1", mask)

        # mask = cv.bitwise_not(mask)
        # cont, cont_frame = self.get_contours(mask)
        # for cnt in cont:
        #     if 5000 > cv.contourArea(cnt):
        #         cv.drawContours(mask, [cnt], 0, self.contour_color, 2)
        #         cv.fillPoly(mask, pts=[cnt], color=self.contour_color)
        #
        # mask = cv.bitwise_not(mask)
        # cv.imshow("fillgaps2", mask)

        mask = self.morpholoical_op(mask, 9, 2, 2)

        # current frame is a 720 x 1280 black frame
        black_frame = np.uint8(np.zeros((720, 1280, 3)))

        contours, contour_frame = self.get_contours(mask)
        cv.drawContours(black_frame, contours, -1, self.contour_color, 3)
        cv.fillPoly(black_frame, pts=contours, color=self.contour_color)
        lines, slopes, ellipse_frame = self.ellipse_slopes(contours, black_frame)

        if show:
            Lines.draw_lines_on_frame(lines, black_frame)

        if len(lines) == 1:
            line = lines[0]
            delta_y = (line[0])
            delta_x = (line[1]-line[3])
            if delta_x != 0:
                angle = round(math.degrees(math.atan(delta_y/delta_x)), 2)
                return black_frame, -angle
            else:
                return black_frame, None
        else:
            return black_frame, None

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

    def morpholoical_op(self, mask, kernel_size, op, iterations):
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        if op == 1:
            mask = cv.erode(mask, kernel, iterations=iterations)
            cv.imshow("erode1", mask)
        if op == 2:
            mask = cv.dilate(mask, kernel, iterations=iterations)
        if op == 3:
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=iterations)
        else:
            mask = mask

        return mask

    def get_contours(self, binary_mask):
        """ Uses the cv.findContours to find the contours in binary_mask
        (creates closed shapes around connected pixels)\n
        :param binary_mask: binary mask of current frame
        :return: list of contours, blank frame with contours filled in
        """
        frame = np.zeros((self.HEIGHT, self.WIDTH, 3))
        ret, thresh = cv.threshold(binary_mask, 0, 254, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        return contours, frame

    def get_biggest_contour(self,contours):
        contour_area = 0
        for cnt in contours:
            if cv.contourArea(cnt)>contour_area:
                contour_area = cv.contourArea(cnt)
                big_contour = cnt
        return cnt

    def ellipse_slopes(self, contours, black_frame):
        """Draws ellipses around each contour on black_frame using cv.fitEllipse,
        Uses cv.fitline with the list of contours to create a set of lines\n
        :param contours: list of contours on the current frame
        :param black_frame: black frame with contours (mat)
        :return: lines (array), slops (array), updated frame with ellipses and lines
        """
        slopes = []
        lines = []
        self.contours = []

        for cnt in contours:
            if self.min_contour_area < cv.contourArea(cnt):
                rect = cv.minAreaRect(cnt)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                black_frame = cv.drawContours(black_frame, [box], 0, (255, 255, 255), 2)
                ellipse = cv.fitEllipse(cnt)
                self.contours.append(ellipse)
                cv.ellipse(black_frame, ellipse, (255, 255, 255), 2)
                rows, cols = black_frame.shape[:2]

                # Defines a line for the contour using the fitLine function
                [vx, vy, x, y] = cv.fitLine(cnt, cv.DIST_L2, 0, 0.01, 0.01)
                # calculates the slope and adds the slope to the array of slopes
                slope = vy / vx
                slopes.append(slope)
                # Finds two other points on the line using the slope
                lefty = int((-x * vy / vx) + y)
                righty = int(((cols - x) * vy / vx) + y)
                black_frame = cv.line(black_frame, (cols - 1, righty), (0, lefty), (255, 255, 0), 9)
                # Appends a line to the lines array using the (x1,y1,x2,y2) definition
                lines.append([cols - 1, righty, 0, lefty])

        return lines, slopes, black_frame

    def get_center_row_coords(self):
        """
        :return: center coordinates of center row contour (x, y) for most recently processed frame. This is different
        from the vanishing point.
        """
        return self.center[0]

    def get_angle_to_minimize(self):
        """
        :return: angle to minimize for most recently processed frame in Radians.
        It is the angle between the vanishing point and the center of the center row contour
        """
        return self.center_angle

    def gaussian_blur(self, frame):
        """Applies gaussian blurring to the frame\n
        :param frame: current frame
        :return: frame with gaussian blurring
        """
        return cv.GaussianBlur(frame, self.gauss_kernel_size, self.sigma_x)
