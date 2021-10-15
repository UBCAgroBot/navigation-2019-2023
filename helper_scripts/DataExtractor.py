import os
import pickle
import sys
import time
from pathlib import Path

import cv2 as cv
import numpy as np


# DataExtractor allows that the user to interact with keyframes in a video by recording and saving data across frames
class DataExtractor:

    def __init__(self, video_name, interval):
        """
        @param video_name (string): name of video
        @param (interval): interval between keyframes
        """
        self.name = video_name
        self.interval = interval
        self.root = f'extract/{self.name}'
        self.keyframes_path = f'{self.root}/keyframes'
        self.video_path = f'{self.root}/video'

        # frames and frames_copy store are a list of keyframes
        # pointer is the index of the current keyframe displayed
        # num_frames is the length of self.frames
        # data is a numpy array storing data for each frame, it is selected by the user for each frame
        self.frames = []
        self.frames_copy = []
        self.pointer = 0
        self.num_frames = 0
        self.data = None

        # create directories
        os.makedirs(self.root)
        os.makedirs(self.keyframes_path)
        os.makedirs(self.video_path)

    def extract(self):
        """Extracts and saves keyframes and video of keyframes to disk. Also updates self to reflect changes."""
        start = time.time()

        stream = cv.VideoCapture(f'videos/{self.name}.mp4')

        success, frame = stream.read()
        h, w, d = frame.shape

        out = cv.VideoWriter(f'{self.video_path}/{self.name}_key.mp4', cv.VideoWriter_fourcc(*'mp4v'), 1, (w, h))

        count = 0
        while success:
            # if frame is a keyframe create a red, vertical line in the middle of the frame and save it
            if count % self.interval == 0:
                frame = cv.line(frame, (w // 2, 0), (w // 2, h), (0, 0, 255), thickness=2)
                self.frames.append(frame)
                self.frames_copy.append(frame.copy())
                out.write(frame)
                cv.imwrite(f'{self.keyframes_path}/{self.name}_{count}.jpg', frame)
                print('frame ', count)
            success, frame = stream.read()
            count += 1

        # save video
        out.release()

        # assign self.data a numpy array of zeroes
        self.num_frames = len(self.frames)
        self.data = np.zeros(self.num_frames)

        end = time.time()

        print('total keyframes :', self.num_frames)
        print(f'data extraction completed in {end - start} secs')

    def select(self):
        """Creates a window displaying the image the pointer points to, sets mouseCallback and starts event loop"""
        cv.namedWindow(self.name)
        cv.imshow(self.name, self.frames[self.pointer])
        cv.setMouseCallback(self.name, self.__click_handler)

        self.__event_loop()

    def save(self):
        """Save self.data, overwrite it if it exists"""
        Path(f'data').mkdir(parents=True, exist_ok=True)
        with open(f'data/{self.name}.pickle', 'wb') as f:
            pickle.dump(self.data, f)
            print(f'file saved at ./data/{self.name}.pickle')

    def __event_loop(self):
        """Carries out task based on keyboard clicks
        a: decrement pointer and show previous image
        d: increment pointer and show next image
        p: dump pickle file
        t: terminate loop"""
        while True:
            key = cv.waitKey(0)

            if key == ord('a'):
                if self.pointer > 0:
                    self.pointer -= 1
                    cv.imshow(self.name, self.frames[self.pointer])
            elif key == ord('d'):
                if self.pointer < self.num_frames - 1:
                    self.pointer += 1
                    cv.imshow(self.name, self.frames[self.pointer])
            elif key == ord('p'):
                self.save()
            elif key == ord('t'):
                message = f'are you sure want to exit it? (y/n)\n'
                confirm = input(message)
                while confirm != 'y' and confirm != 'n':
                    print('invalid input')
                    confirm = input(message)
                if confirm == 'n':
                    sys.exit()

    def __click_handler(self, event, x, y, flags, param):
        """If left-click is detected, restore original frame, draw a blue circle at (x, y) and update self.data
        @param event (event type enum): contains the type of event
        @param (x): x-coordinate of mouse
        @param (y): y-coordinate of mouse
        """
        if event == cv.EVENT_LBUTTONDOWN:
            self.frames[self.pointer] = self.frames_copy[self.pointer].copy()
            cv.circle(self.frames[self.pointer], (x, y), 3, (255, 0, 0), -1)
            self.data[self.pointer] = x
            print(f'frame {self.pointer}, (x, y) = {(x, y)}')
            print(self.data)
            cv.imshow(self.name, self.frames[self.pointer])
