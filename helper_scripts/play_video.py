from tkinter.filedialog import askopenfilename
from tkinter import Tk     # from tkinter import Tk for Python 3.x
import cv2
<< << << < HEAD
video = '../videos/grape.mp4'
== == == =
>>>>>> > 3f9053bf5d3251d9cbdfe2ffe8a86630cba44353

if __name__ == '__main__':
    Tk().withdraw()  # we don't want a full GUI, so keep the root window from appearing
    # show an "Open" dialog box and return the path to the selected file
    video_file = askopenfilename(initialdir='../videos')

    vid = cv2.VideoCapture(video_file)

    video_name = video_file.split('/')[-1].split('.')[0]
    if (vid.isOpened() == False):
        print("Error Opening Video File")

    while (vid.isOpened()):
        ret, frame = vid.read()

        if not ret:
            print("No More Frames Remaining")
            break
        print(frame.shape)
        cv2.imshow(video_name, frame)
        key = cv2.waitKey(25)

        # Exit if Esc key is pressed
        if key == 27:
            break
        # save frame if 's' key is pressed
        if key == ord('s'):
            cv2.imwrite('%s.jpg' % video_name, frame)
            print('current frame saved')
            cv2.waitKey(2000)

    vid.release()
    cv2.destroyAllWindows()
