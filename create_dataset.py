import argparse
import os.path as path
import shutil
import sys

from helper_scripts.DataExtractor import DataExtractor

# parser for command line arguments
# -v/--vid: (string) name of video stored in ./videos
# -i/--interval: (integer) keyframe interval
# -o/-overwrite: (boolean) flag to overwrite files in ./extract/vid, default=false
parser = argparse.ArgumentParser()
parser.add_argument('-v', '--vid', required=True)
parser.add_argument('-i', '--interval', default='30')
parser.add_argument('-o', '--overwrite', action='store_true')


def main():
    """Accepts arguments and creates DataExtractor object"""
    video_name = args.vid
    frame_interval = int(args.interval)
    overwrite = args.overwrite

    # verify that video exists in ./videos
    if not path.isfile(f'videos/{video_name}.mp4'):
        print('--vid', video_name, "is an invalid video name, make sure video exists in ./videos")
        sys.exit()

    # make sure any important data does not get overwritten
    if path.isfile(f'data/{video_name}.pickle'):
        message = f'data/{video_name}.pickle already exists, are you sure want to overwrite it? (y/n)\n'
        confirm = input(message)
        while confirm != 'y' and confirm != 'n':
            print('invalid input')
            confirm = input(message)
        if confirm == 'n':
            sys.exit()

    # delete ./extract/{video_name} folder
    if overwrite:
        shutil.rmtree(f'extract/{video_name}', ignore_errors=True)

    # initialize DataExtractor object and call its methods
    data_extractor = DataExtractor(video_name, frame_interval)
    data_extractor.extract()
    data_extractor.select()


if __name__ == '__main__':
    args = parser.parse_args()
    main()
