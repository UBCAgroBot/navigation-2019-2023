import math

import cv2 as cv
import numpy as np
from algorithms.Algorithm import Algorithm
from algorithms.utils import Lines
from algorithms.utils import delete_small_contours


class CenterDownward(Algorithm):

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

        mask = self.create_binary_mask(frame)
        mask = delete_small_contours.morph_op(mask, 9, 2, 1)

        cont, cont_frame = self.get_contours(mask)

        for cnt in cont:
            if 5000 > cv.contourArea(cnt):
                cv.drawContours(mask, [cnt], 0, self.contour_color, 2)
                cv.fillPoly(mask, pts=[cnt], color=self.contour_color)

        mask = delete_small_contours.morph_op(mask, 9, 2, 2)

        # current frame is a 720 x 1280 black frame
        black_frame = np.uint8(np.zeros((720, 1280, 3)))

        contours, contour_frame = self.get_contours(mask)
        cv.drawContours(black_frame, contours, -1, self.contour_color, 3)
        cv.fillPoly(black_frame, pts=contours, color=self.contour_color)

        if contours is ():
            return black_frame, None

        lines, slopes, ellipse_frame = self.ellipse_slopes(contours, black_frame)

        if show:
            Lines.draw_lines_on_frame(lines, black_frame)

        if len(lines) == 1:
            line = lines[0]
            delta_y = (line[0])
            delta_x = (line[3] - line[1])

            if line[0] - line[2] == 0:
                return black_frame, 0
            elif delta_x != 0:
                angle = round(math.degrees(math.atan(delta_y / delta_x)), 2)
                return black_frame, angle
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

    def get_biggest_contour(self, contours):
        contour_area = 0

        for cnt in contours:
            if cv.contourArea(cnt) > contour_area:
                contour_area = cv.contourArea(cnt)
                big_contour = cnt
        return big_contour

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

        con = self.get_biggest_contour(contours)
        rect = cv.minAreaRect(con)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        black_frame = cv.drawContours(black_frame, [box], 0, (255, 255, 255), 2)
        ellipse = cv.fitEllipse(con)
        self.contours.append(ellipse)
        cv.ellipse(black_frame, ellipse, (255, 255, 255), 2)

        point1 = box[0]
        point2 = box[3]
        point3 = box[2]

        if math.sqrt((point2[-2] - point1[-2]) ** 2 + (point2[-1] - point1[-1]) ** 2) > math.sqrt(
                (point3[-2] - point2[-2]) ** 2 + (point3[-1] - point2[-1]) ** 2):

            vy = (point2[-1] - point1[-1])
            vx = (point2[-2] - point1[-2])

        else:

            vy = (point3[-1] - point2[-1])
            vx = (point3[-2] - point2[-2])

        point = rect[-3]
        length = math.sqrt(vx ** 2 + vy ** 2)

        width, height = box[-2]
        x = point[0]
        y = point[1]

        rows, cols = black_frame.shape[:2]

        if vx != 0:
            slope = vy / vx
            slopes.append(slope)
            lefty = int((-x * vy / vx) + y)
            righty = int(((cols - x) * vy / vx) + y)
            black_frame = cv.line(black_frame, (cols - 1, righty), (0, lefty), (255, 255, 0), 9)
            lines.append([cols - 1, righty, 0, lefty])

        else:
            black_frame = cv.line(black_frame, (int(x), 0), (int(x), rows - 1), (255, 255, 0), 9)
            lines.append([int(x), 0, int(x), rows - 1])

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
