from cv2 import cv2
import numpy as np
import hough_algorithm
import contour_algorithm
import mini_contour_algorithm
<<<<<<< HEAD
vid = cv2.VideoCapture("videos/cropVid2.mp4")
 
if(vid.isOpened() == False):
    print("Error Opening Video File")
=======

vid = cv2.VideoCapture("/Users/rithin/PycharmProjects/Agrobots/videos/cropVid2.mp4")
>>>>>>> origin/RithinBranch

if (vid.isOpened() == False):
    print("Error Opening Video File")

while (vid.isOpened()):
    ret, frame = vid.read()
    if ret == False:
        print("No More Frames Remaining")
        break
<<<<<<< HEAD
    
    ################### ADD ALGORITHM HERE ###################
    h = hough_algorithm.hough_algorithm()
    #h = contour_algorithm.contour_algorithm()
    #h = mini_contour_algorithm.mini_contour_algorithm()
    h.process_frame(frame)
=======
>>>>>>> origin/RithinBranch

    ################### ADD ALGORITHM HERE ###################
    # Uncomment the required algorithm
    a = hough_algorithm.hough_algorithm()
    # b = contour_algorithm.contour_algorithm()
    # c = mini_contour_algorithm.MiniContoursAlgorithm()
    a.processFrame(frame)
    # b.processFrame(frame)
    # c.processFrame(frame)

    key = cv2.waitKey(25)
    # If Enter is pressed capture current frames
    if key == 13:
        # Save Current Frames and Exit
        cv2.imwrite('original.png', frame)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imwrite('colour_mask.png', mask)
        cv2.imwrite('colour_res.png', res)
        cv2.imwrite('edges.png', edges)
        cv2.imwrite('lineimg.png', lineimg)
    # Exit if Esc key is pressed
    if key == 27:
        break

vid.release()
cv2.destroyAllWindows()
