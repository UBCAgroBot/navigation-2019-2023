import cv2 as cv


def change_res(frame, scale_percent):
    scale_percent = scale_percent
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)

    frame = cv.resize(frame, dim, interpolation=cv.INTER_AREA)

    # scale_percent = 10000/scale_percent
    # width = int(frame.shape[1] * scale_percent / 100)
    # height = int(frame.shape[0] * scale_percent / 100)
    # dim = (width, height)
    #
    # frame = cv.resize(frame, dim, interpolation=cv.INTER_AREA)

    return frame


def change_res_2(frame, resolution):
    height, width, channels = frame.shape
    new_width = int(width/height*resolution)
    frame = cv.resize(frame, (new_width, resolution), interpolation=cv.INTER_AREA)

    return frame
