import cv2 as cv


def change_res(frame, scale_percent):
    scale_percent = scale_percent
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)

    frame = cv.resize(frame, dim, interpolation=cv.INTER_AREA)

    return frame
