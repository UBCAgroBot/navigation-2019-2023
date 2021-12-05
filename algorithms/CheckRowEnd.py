import cv2 as cv
import numpy as np


class CheckRowEnd:

    def __init__(self, config):
        """Sets checkRowEnd algorithm configurations\n
        :param config: config params
        """

        # mask thresholds
        self.low_green = np.array(config.lower_hsv_threshold)
        self.high_green = np.array(config.upper_hsv_threshold)

        # Percentage of empty rows required to register as row end
        self.percentage_of_empty_rows = config.percentage_of_empty_rows

    def processFrame(self, frame, show=True):
        """Averages values in each row in a mask of the frame. If the number of rows with an average value
        is greater than req_rows_empty, then frame is row end\n
        :param frame: current frame (mat)
        :param show: show/hide frames on screen for debugging
        :type show: bool
        :return: mask (2d array), is_end (bool)
        """

        # binary mask
        mask = self.create_binary_mask(frame)

        # number of required empty rows for frame to be declared as row end
        rows, cols = mask.shape
        req_rows_empty = rows * self.percentage_of_empty_rows

        is_end = False
        if rows - np.count_nonzero(mask.mean(axis=1)) > req_rows_empty:
            is_end = True
            print('end of row reached')

        return mask, is_end

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
        mask = cv.inRange(hsv, self.low_green, self.high_green)

        return mask
