import os
import time

import cv2 as cv


class DataExtractor:

    def __init__(self, video_name, interval):
        self.video = video_name
        self.interval = interval
        self.root = f'extract/{self.video}'
        self.keyframes_path = f'{self.root}/keyframes'
        self.video_path = f'{self.root}/video'

        os.makedirs(self.root)
        os.makedirs(self.keyframes_path)
        os.makedirs(self.video_path)

    def extract(self):
        start = time.time()

        stream = cv.VideoCapture(f'videos/{self.video}.mp4')

        success, frame = stream.read()
        h, w, d = frame.shape

        out = cv.VideoWriter(f'{self.video_path}/{self.video}_key.mp4', cv.VideoWriter_fourcc(*'mp4v'), 1, (w, h))

        count = 0
        while success:
            if count % self.interval == 0:
                frame = cv.line(frame, (w // 2, 0), (w // 2, h), (0, 0, 255), thickness=2)
                out.write(frame)
                cv.imwrite(f'{self.keyframes_path}/{self.video}_{count}.jpg', frame)
                print('frame ', count)
            success, frame = stream.read()
            count += 1

        out.release()
        end = time.time()

        print('total keyframes :', count // 30 + 1)
        print(f'data extraction completed in {end - start} secs')

    def select(self):
        pass

    def save(self):
        pass
