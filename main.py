from cv2 import cv2
import numpy as np
import hough_algorithm
import contour_algorithm
import mini_contour_algorithm

vid = cv2.VideoCapture("/home/davidw0311/AgroBot/videos/cropVid2.mp4")

if (vid.isOpened() == False):
    print("Error Opening Video File")

while (vid.isOpened()):
    ret, frame = vid.read()
    if ret == False:
        print("No More Frames Remaining")
        break

    ################### ADD ALGORITHM HERE ###################
    # Uncomment the required algorithm
    a = hough_algorithm.hough_algorithm()
    # b = contour_algorithm.contour_algorithm()
    # c = mini_contour_algorithm.MiniContoursAlgorithm()
    processed_image, intersection_point = a.processFrame(frame)
    # b.processFrame(frame)
    # c.processFrame(frame)

    cv2.imshow('processed image', processed_image)
    key = cv2.waitKey(25)
    print('intersection point', intersection_point)
    # Exit if Esc key is pressed
    if key == 27:
        break

vid.release()
cv2.destroyAllWindows()
