import cv2 as cv
<<<<<<< HEAD
video = '../videos/grape.mp4'
=======
from tkinter import Tk     # from tkinter import Tk for Python 3.x
from tkinter.filedialog import askopenfilename
>>>>>>> 3f9053bf5d3251d9cbdfe2ffe8a86630cba44353

if __name__ == '__main__':
    Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    video_file = askopenfilename(initialdir='../videos') # show an "Open" dialog box and return the path to the selected file
    
    vid = cv.VideoCapture(video_file)

    video_name = video_file.split('/')[-1].split('.')[0]
    if (vid.isOpened() == False):
        print("Error Opening Video File")

    while (vid.isOpened()):
        ret, frame = vid.read()
        
        if ret == False:
            print("No More Frames Remaining")
            break
        print(frame.shape)
        cv.imshow(video_name, frame)
        key = cv.waitKey(25)

        #### Exit if Esc key is pressed
        if key == 27:
            break
        #### save frame if 's' key is pressed
        if key == ord('s'):
            cv.imwrite('%s.jpg' % video_name, frame)
            print('current frame saved')
            cv.waitKey(2000)

    vid.release()
    cv.destroyAllWindows()