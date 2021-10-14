import cv2
video = '../videos/grape.mp4'

if __name__ == '__main__':

    vid = cv2.VideoCapture(video)

    if (vid.isOpened() == False):
        print("Error Opening Video File")

    while (vid.isOpened()):
        ret, frame = vid.read()
        if ret == False:
            print("No More Frames Remaining")
            break

        cv2.imshow("video", frame)
        key = cv2.waitKey(25)

        #### Exit if Esc key is pressed
        if key == 27:
            break
        #### save frame if 's' key is pressed
        if key == ord('s'):
            cv2.imwrite('saved_frame.jpg', frame)
            print('current frame saved')
            cv2.waitKey(2000)

    vid.release()
    cv2.destroyAllWindows()