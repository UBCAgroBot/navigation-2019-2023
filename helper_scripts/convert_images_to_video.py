'''
This script takes in a folder of images and converts to a video
'''

import cv2 
import numpy as np
import glob

IMAGES_FOLDER = '/home/ubcagrobot/Documents/field_videos/test/frames'
OUTPUT_FILE = '/home/ubcagrobot/Documents/field_videos/test.avi'
img_array = []

for filename in glob.glob(f'{IMAGES_FOLDER}/*.png'):
    print(filename)

    img = cv2.imread(filename)
    (h,w,layers) = img.shape
    size = (w,h)
    img_array.append(img)

out = cv2.VideoWriter(OUTPUT_FILE, cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

for i in range(len(img_array)):
    out.write(img_array[i])
out.release()