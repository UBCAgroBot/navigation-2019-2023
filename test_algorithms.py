import sys
import argparse 
from cv2 import cv2
import numpy as np
import yaml
from algorithms.CenterRowAlgorithm import CenterRowAlgorithm
from algorithms.HoughAlgorithm import HoughAlgorithm
from algorithms.MiniContoursAlgorithm import MiniContoursAlgorithm
from algorithms.ScanningAlgorithm import ScanningAlgorithm

from omegaconf import OmegaConf 
parser = argparse.ArgumentParser()
parser.add_argument('--alg', default='hough')
parser.add_argument('--vid', default='crop')

def main():
        
    if args.vid == 'crop':
        print('Using Crop Video')
        vid_config = OmegaConf.load('config/video/crop.yaml')
        vid_file = "videos/crop.mp4"
    elif args.vid == 'sim':
        print('Using Sim Video')
        vid_config = OmegaConf.load('config/video/sim.yaml')
        vid_file = "videos/sim.mp4"
    elif args.vid == 'grape':
        print('Using Grape Video')
        vid_config = OmegaConf.load('config/video/grape.yaml')
        vid_file = "videos/grape.mp4"
    else:
        print('--vid', args.vid, "is not defined, specify one of 'crop' or 'sim'")
        sys.exit()

    if args.alg == 'hough':
        print('Using Hough Algorithm')
        alg_config = OmegaConf.load('config/algorithm/hough.yaml')
        config = OmegaConf.merge(alg_config, vid_config)
        alg = HoughAlgorithm(config)
    elif args.alg == 'center_row':
        print('Using Center Row Algorithm')
        alg_config = OmegaConf.load('config/algorithm/center_row.yaml')
        config = OmegaConf.merge(alg_config, vid_config)
        alg = CenterRowAlgorithm(config)
    elif args.alg == 'mini_contour':
        print('Using Mini Contours Algorithm')
        alg_config = OmegaConf.load('config/algorithm/mini_contour.yaml')
        config = OmegaConf.merge(alg_config, vid_config)
        print(config)
        alg = MiniContoursAlgorithm(config)
    elif args.alg == 'scanning':
        print("Using Scanning Algorithm")
        alg_config = OmegaConf.load('config/algorithm/scanning.yaml')
        print(alg_config)
        print(vid_config)
        config = OmegaConf.merge(alg_config, vid_config)
        alg = ScanningAlgorithm(config)
    else:
        print('--alg', args.alg, "is not defined, specify one of: 'hough', 'center_row', 'mini_contour', or 'scanning'")
        sys.exit()


    run_algorithm(alg, vid_file)

def run_algorithm(alg, vid_file):
    vid = cv2.VideoCapture(vid_file)

    if (vid.isOpened() == False):
        print("Error Opening Video File")

    while (vid.isOpened()):
        ret, frame = vid.read()
        if ret == False:
            print("No More Frames Remaining")
            break

        processed_image, intersection_point = alg.processFrame(frame, show=True)

        cv2.imshow("%s agorithm on %s video"%(args.alg, args.vid), processed_image)
        key = cv2.waitKey(25)

        # Exit if Esc key is pressed
        if key == 27:
            break

    vid.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    args = parser.parse_args()
    main()
    

    




