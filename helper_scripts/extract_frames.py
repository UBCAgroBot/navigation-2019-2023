import os

import cv2 as cv

path = '../videos/'
file = 'sim_trimmed'
file_type = '.mp4'

os.makedirs('../keyframes/' + file)

vid = cv.VideoCapture(path + file + file_type)
success, frame = vid.read()
count = 0

h, w, l = frame.shape
out = cv.VideoWriter(path + file + '_keyframes' + file_type, cv.VideoWriter_fourcc(*'mp4v'), 1, (w, h))

while success:
    if count % 30 == 0:
        out.write(frame)
        cv.imwrite('../keyframes/' + file + '/frame%d.jpg' % count, frame)
        print('frame ', count)
    success, frame = vid.read()
    count += 1

out.release()
print('total keyframes :', count // 30 + 1)
