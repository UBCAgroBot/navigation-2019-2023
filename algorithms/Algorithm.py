import numpy as np


class Algorithm:
    def __init__(self, config) -> None:
        self.low_green = np.array(config.lower_hsv_threshold)
        self.high_green = np.array(config.upper_hsv_threshold)
