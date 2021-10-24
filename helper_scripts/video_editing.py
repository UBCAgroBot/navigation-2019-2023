import cv2
from tkinter import Tk     # from tkinter import Tk for Python 3.x
from tkinter.filedialog import askopenfilename

Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
video_file = askopenfilename(initialdir='../videos') # show an "Open" dialog box and return the path to the selected file

print('video_file', video_file)
vid = cv2.VideoCapture(video_file)

video_name = video_file.split('/')[-1].split('.')[0]

def save_rotated_video():

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
       
    out = cv2.VideoWriter('edited.mp4', fourcc, 20.0, (1920,1080))
    if (vid.isOpened() == False):
        print("Error Opening Video File")

    while (vid.isOpened()):
        ret, frame = vid.read()
        if ret == False:
            print("No More Frames Remaining")
            break
        
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow('rotate frame', frame)
        cv2.waitKey(1)
        out.write(frame)
    
    vid.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    save_rotated_video()

    