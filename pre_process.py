import cv2
import math

BRIGHTNESS_BASELINE = 110
SATURATION_BASELINE = 105
ACCEPTABLE_DIFFERENCE = 25


# Calls helper functions to increase or decrease brightness 
# of an image based on the amount parameter
def alter_brightness(img, amount):
    # (Image, int) -> Image
    if amount > 0:
        return increase_brightness(img, amount)
    elif amount < 0:
        return decrease_brightness(img, -1 * amount)
    else:
        return img


# Calls helper functions to increase or decrease saturation 
# of an image based on the amount parameter
def alter_saturation(img, amount):
    # (Image, int) -> Image
    if amount > 0:
        return increase_saturation(img, amount)
    elif amount < 0:
        return decrease_saturation(img, -1 * amount)
    else:
        return img


# Decreases the brightness (v in hsv) for each pixel by given amount
def decrease_brightness(img, value):
    # (Image, int) -> Image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    v[v < value] = 0
    v[v >= value] -= value

    final_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)


# Increases the brightness (v in hsv) for each pixel by given amount
def increase_brightness(img, value):
    # (Image, int) -> Image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)


# Decreases the saturation (s in hsv) for each pixel by given amount
def decrease_saturation(img, value):
    # (Image, int) -> Image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    s[s < value] = 0
    s[s >= value] -= value

    final_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)


# Increases the saturation (s in hsv) for each pixel by given amount
def increase_saturation(img, value):
    # (Image, int) -> Image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    s[s > lim] = 255
    s[s <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)


# Main function that modifies the saturation and brightness of a frame based on
# it's mean s and v values
def standardize_frame(img):
    # Image -> Image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mean_brightness = hsv[..., 2].mean()
    mean_saturation = hsv[..., 1].mean()

    if mean_brightness < BRIGHTNESS_BASELINE:
        if mean_brightness + ACCEPTABLE_DIFFERENCE < BRIGHTNESS_BASELINE:
            img = alter_brightness(img, math.floor(BRIGHTNESS_BASELINE - (mean_brightness + ACCEPTABLE_DIFFERENCE)))
    else:
        if mean_brightness - ACCEPTABLE_DIFFERENCE > BRIGHTNESS_BASELINE:
            img = alter_brightness(img, math.floor(BRIGHTNESS_BASELINE - (mean_brightness - ACCEPTABLE_DIFFERENCE)))
    if mean_saturation < SATURATION_BASELINE:
        if mean_saturation + ACCEPTABLE_DIFFERENCE < SATURATION_BASELINE:
            img = alter_saturation(img, math.floor(SATURATION_BASELINE - (mean_saturation + ACCEPTABLE_DIFFERENCE)))
    else:
        if mean_saturation - ACCEPTABLE_DIFFERENCE > SATURATION_BASELINE:
            img = alter_saturation(img, math.floor(SATURATION_BASELINE - (mean_saturation - ACCEPTABLE_DIFFERENCE)))

    return img
