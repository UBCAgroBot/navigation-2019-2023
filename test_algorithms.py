import argparse
import os.path as path
import sys

import cv2 as cv
from omegaconf import OmegaConf

from algorithms.CenterRowAlgorithm import CenterRowAlgorithm
from algorithms.CheckRowEnd import CheckRowEnd
from algorithms.HoughAlgorithm import HoughAlgorithm
from algorithms.MiniContoursAlgorithm import MiniContoursAlgorithm
from algorithms.MiniContoursDownwards import MiniContoursDownwards
from algorithms.ScanningAlgorithm import ScanningAlgorithm
import pre_process
from algorithms.SeesawAlgorithm import SeesawAlgorithm
from algorithms.CenterDownwards import CenterDownward

# parser for command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--alg', required=True)
parser.add_argument('-v', '--vid', required=True)
parser.add_argument('-s', '--show', required=False, action='store_true')

# list of algorithms
algo_list = [('hough', HoughAlgorithm), ('center_row', CenterRowAlgorithm), ('mini_contour', MiniContoursAlgorithm),
             ('mini_contour_downward', MiniContoursDownwards),
             ('scanning', ScanningAlgorithm), ('check_row_end', CheckRowEnd), ('seesaw', SeesawAlgorithm), ("center_down", CenterDownward)]


def main():
    # verify that video exists in ./videos
    if not path.isfile(f'videos/{args.vid}.mp4'):
        print('--vid', args.vid,
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

    # set video config
    vid_config = OmegaConf.load(f'config/video/{args.vid}.yaml')
    vid_file = f"videos/{args.vid}.mp4"

    # set algorithm config
    alg_config = OmegaConf.load(f'config/algorithm/{args.alg}.yaml')

    # merge config files
    config = OmegaConf.merge(alg_config, vid_config)

    # run algorithm if it exists in algo_list, else print error message and exit
    exists = False
    for elem in algo_list:
        if elem[0] == args.alg:
            alg = elem[1](config)
            run_algorithm(alg, vid_file)
            exists = True

    if not exists:
        print(
            f"{args.alg} is an invalid algorithm, list of valid argument values: {algo_list}")
        sys.exit()


def run_algorithm(alg, vid_file):
    vid = cv.VideoCapture(vid_file)

    if not vid.isOpened():
        print('Error Opening Video File')

    while vid.isOpened():
        ret, frame = vid.read()

        if not ret:
            print('No More Frames Remaining')
            break

        frame = pre_process.standardize_frame(frame)
        processed_image, angle = alg.process_frame(frame, show=args.show)

        print(angle)

        if args.show:
            cv.imshow(
                f'{args.alg}s algorithm on {args.vid}s video', processed_image)

        key = cv.waitKey(25)

        # Exit if Esc key is pressed
        if key == 27:
            break

    vid.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    args = parser.parse_args()
    main()
