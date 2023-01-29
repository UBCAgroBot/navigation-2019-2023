import argparse
import os.path as path
import sys
import time

import cv2 as cv
import numpy as np
from omegaconf import OmegaConf

from algorithms.CenterRowAlgorithm import CenterRowAlgorithm
from algorithms.CheckRowEnd import CheckRowEnd
from algorithms.HoughAlgorithm import HoughAlgorithm
from algorithms.MiniContoursAlgorithm import MiniContoursAlgorithm
from algorithms.MiniContoursDownwards import MiniContoursDownwards
from algorithms.ScanningAlgorithm import ScanningAlgorithm

# parser for command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--alg', required=True)
parser.add_argument('-v', '--vid', required=True)
parser.add_argument('-s', '--show', required=False, action='store_true')

# list of algorithms
algo_list = [
    ('hough',
     HoughAlgorithm),
    ('center_row',
     CenterRowAlgorithm),
    ('mini_contour',
     MiniContoursAlgorithm),
    ('mini_contour_downward',
     MiniContoursDownwards),
    ('scanning',
     ScanningAlgorithm),
    ('check_row_end',
     CheckRowEnd)]
#
frames_for_GUI = ""

# ability to pass in args for reusability
def main(args):
    # verify that video exists in ./videos
    if not path.isfile(f'videos/{args.vid}.mp4'):
        print(
            '--vid',
            args.vid,
            "is an invalid video name, make sure it video exists in ./videos")
        sys.exit()

    # verify that config file for video exists
    if not path.isfile(f'config/video/{args.vid}.yaml'):
        print(f"--alg config for {args.vid} is not defined in ./config/video/")
        sys.exit()

    # verify that config file for algorithm exists
    if not path.isfile(f'config/algorithm/{args.alg}.yaml'):
        print(
            f"--alg config for {args.alg} is not defined in ./config/algorithm/")
        sys.exit()

    # set video and algorithm config, then merge
    vid_config = OmegaConf.load(f'config/video/{args.vid}.yaml')
    vid_file = f"videos/{args.vid}.mp4"
    alg_config = OmegaConf.load(f'config/algorithm/{args.alg}.yaml')
    config = OmegaConf.merge(alg_config, vid_config)

    # find if algorithm exists in algo_list
    alg = None
    for elem in algo_list:
        if elem[0] == args.alg:
            alg = elem[1](config)
            break

    # using time package to time the start point
    start_time = time.time()

    # run algorithm if it exists, else return an error
    if alg is not None:
        uptime, frameCount, all_time = run_algorithm(alg, vid_file)
    else:
        print(
            f"{args.alg} is an invalid algorithm, list of valid argument values: {algo_list}")
        sys.exit()

    # using time package to time the end point
    end_time = time.time()
    # display time till processed all frames in a video
    print(
        "time till finish execution: %2.2f sec" % (end_time - start_time),
        "\npercentage time of a valid return from process_frame: %.2f%%" % (
            100. * uptime / frameCount),
        "\naverage time to process a frame: %2.2f sec" % (
            1. * sum(all_time) / len(all_time)),
        "\nframe per second: %2.2f fps" % (
            1. * (frameCount / (end_time - start_time))),
        "\n"
    )



# copied from test_algorithms.py
# runs the algorithm on each frame of video, count the vanishing point uptime
def run_algorithm(alg, vid_file):
    vid = cv.VideoCapture(vid_file)

    if not vid.isOpened():
        print('Error Opening Video File')

    total_run = 0
    uptime = 0
    all_frame_time = []
    #
    global frames_for_GUI
    frames_for_GUI = {}
    i = 0
    #
    while vid.isOpened():
        ret, frame = vid.read()
        if not ret:
            print('No More Frames Remaining\n')
            break

        start_time_frame = time.time()
        binary, mask, ctrs, angle = alg.process_frame(frame, show=args.show)
        end_time_frame = time.time()
        all_frame_time.append(end_time_frame - start_time_frame)

        # print(angle)

        # counters
        if angle is not None:
            uptime += 1
        total_run += 1

        if args.show:
            cv.imshow(
                f'{args.alg}s algorithm on {args.vid}s video', binary)
            frames_for_GUI.update({"binary"+str(i): binary})
            frames_for_GUI.update({"mask"+str(i): mask})
            frames_for_GUI.update({"ctrs"+str(i): ctrs})
            i += 1

        key = cv.waitKey(1) 
        if key == 27:
            break

    vid.release()
    cv.destroyAllWindows()
    return uptime, total_run, all_frame_time

# display binary_mask, black, contours ...
# TODO: add controller for index of frame
# TODO: add pause/resume functionality
# TODO: add controller after pause

def update_view(val):
    global frames_for_GUI

    if val == 0:
        img = frames_for_GUI['binary10']
    elif val == 1:
        img = frames_for_GUI['mask10']
    elif val == 2:
        img = frames_for_GUI['ctrs10']
    
    cv.imshow('Window', img)
    

def init_gui():
    cv.namedWindow('Window')
    cv.createTrackbar('Toggle View', 'Window', 0, 2, update_view)

    while True:
        key = cv.waitKey(1)
        if key == 27:
            break
    
    cv.destroyAllWindows()

    


if __name__ == '__main__':
    args = parser.parse_args()
    main(args)
    if args.show:
        init_gui()