import math

import cv2
import numpy as np

BRIGHTNESS_BASELINE = 110
SATURATION_BASELINE = 105
ACCEPTABLE_DIFFERENCE = 25


def alter_brightness(img, amount):
    """
        Calls helper functions to increase or decrease brightness
        of an image based on the amount parameter.

        Parameters
        ----------
        img : Image (np.ndarray)
        amount : int

        Returns
        ----------
        Image (np.ndarray), altered brightness on image
    """
    if amount > 0:
        return increase_brightness(img, amount)
    elif amount < 0:
        return decrease_brightness(img, -1 * amount)
    else:
        return img


def alter_saturation(img, amount):
    """
        Calls helper functions to increase or decrease saturation
        of an image based on the amount parameter.

        Parameters
        ----------
        img : Image (np.ndarray)
        amount : int

        Returns
        ----------
        Image (np.ndarray), altered saturation on image
    """
    if amount > 0:
        return increase_saturation(img, amount)
    elif amount < 0:
        return decrease_saturation(img, -1 * amount)
    else:
        return img


def decrease_brightness(img, value):
    """
        Decreases the brightness (v in hsv) for each pixel by given value.

        Parameters
        ----------
        img : Image (np.ndarray)
        value : int

        Returns
        ----------
        Image (np.ndarray), decreased brightness on image
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    v[v < value] = 0
    v[v >= value] -= value

    final_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)


def increase_brightness(img, value):
    """
        Increases the brightness (v in hsv) for each pixel by given value.

        Parameters
        ----------
        img : Image (np.ndarray)
        value : int

        Returns
        ----------
        Image (np.ndarray), increased brightness on image
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)


def decrease_saturation(img, value):
    """
        Decreases the saturation (s in hsv) for each pixel by given value.

        Parameters
        ----------
        img : Image (np.ndarray)
        value : int

        Returns
        ----------
        Image (np.ndarray), decreased saturation on image
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    s[s < value] = 0
    s[s >= value] -= value

    final_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)


def increase_saturation(img, value):
    """
        Increases the saturation (s in hsv) for each pixel by given value.

        Parameters
        ----------
        img : Image (np.ndarray)
        value : int

        Returns
        ----------
        Image (np.ndarray), increased saturation on image
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    s[s > lim] = 255
    s[s <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)


def standardize_frame(img):
    if len(img.shape) < 3:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    try:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    except:
        img_float32 = np.float32(img)
        hsv = cv2.cvtColor(img_float32, cv2.COLOR_BGR2HSV)

    """
        Main function that modifies the saturation and brightness of a frame
        based on it's mean s and v values.

        Parameters
        ----------
        img : Image (np.ndarray)

        Returns
        ----------
        Image (np.ndarray)
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mean_brightness = hsv[..., 2].mean()
    mean_saturation = hsv[..., 1].mean()

    if mean_brightness < BRIGHTNESS_BASELINE:
        if mean_brightness + ACCEPTABLE_DIFFERENCE < BRIGHTNESS_BASELINE:
            img = alter_brightness(img, math.floor(
                BRIGHTNESS_BASELINE - (mean_brightness + ACCEPTABLE_DIFFERENCE)))
    else:
        if mean_brightness - ACCEPTABLE_DIFFERENCE > BRIGHTNESS_BASELINE:
            img = alter_brightness(img, math.floor(
                BRIGHTNESS_BASELINE - (mean_brightness - ACCEPTABLE_DIFFERENCE)))
    if mean_saturation < SATURATION_BASELINE:
        if mean_saturation + ACCEPTABLE_DIFFERENCE < SATURATION_BASELINE:
            img = alter_saturation(img, math.floor(
                SATURATION_BASELINE - (mean_saturation + ACCEPTABLE_DIFFERENCE)))
    else:
        if mean_saturation - ACCEPTABLE_DIFFERENCE > SATURATION_BASELINE:
            img = alter_saturation(img, math.floor(
                SATURATION_BASELINE - (mean_saturation - ACCEPTABLE_DIFFERENCE)))
            img = alter_brightness(img, math.floor(
                BRIGHTNESS_BASELINE - (mean_brightness + ACCEPTABLE_DIFFERENCE)))
        else:
            if mean_brightness - ACCEPTABLE_DIFFERENCE > BRIGHTNESS_BASELINE:
                img = alter_brightness(img, math.floor(
                    BRIGHTNESS_BASELINE - (mean_brightness - ACCEPTABLE_DIFFERENCE)))
    if mean_saturation < SATURATION_BASELINE:
        if mean_saturation + ACCEPTABLE_DIFFERENCE < SATURATION_BASELINE:
            img = alter_saturation(img, math.floor(
                SATURATION_BASELINE - (mean_saturation + ACCEPTABLE_DIFFERENCE)))
    else:
        if mean_saturation - ACCEPTABLE_DIFFERENCE > SATURATION_BASELINE:
            img = alter_saturation(img, math.floor(
                SATURATION_BASELINE - (mean_saturation - ACCEPTABLE_DIFFERENCE)))

    return img
