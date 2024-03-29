import cv2 as cv
import numpy as np

"""
takes in contours and a threshold area, and draw only contours that are bigger than the threshold
contours: a tuple of points that describe a contour, as outputted by the findcontours function in opencv
threshold: the minimum area of a contour for it to be kept
"""


def delete_small_contours(contours, threshold):
    black_frame = np.uint8(np.zeros((720, 1280, 3)))

    for cnt in contours:
        if threshold > cv.contourArea(cnt):
            cv.drawContours(black_frame, [cnt], 0, (0, 129, 255), 2)
            cv.fillPoly(black_frame, pts=[cnt], color=(0, 129, 255))

    return black_frame


"""
morphological operations for image editing
kernel size: integer size of kernel used for image filtering, should be odd number
op: 1 is erode, 2 is dilate, 3 is closing holes, 4 is getting rid of small contours
iterations: the amount of time the operation runs
"""


def morph_op(mask, kernel_size, op, iterations):

    kernel = np.ones((kernel_size, kernel_size), np.uint8)

    if op == 1:
        mask = cv.erode(mask, kernel, iterations=iterations)
    if op == 2:
        mask = cv.dilate(mask, kernel, iterations=iterations)
    if op == 3:
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=iterations)
    if op == 4:
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=iterations)
    else:
        mask = mask

    return mask
