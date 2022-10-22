import numpy as np


class Algorithm:
    def process_frame(self, frame: np.ndarray, show: bool) -> tuple[np.ndarray, float | None]:
        '''
        Processes a frame using the algorithm
        :param frame: current frame (mat)
        :param show: show/hide frames on screen for debugging
        :type show: bool
        :return: processed frame (mat), angle [-90, 90]
        '''
        raise NotImplementedError
