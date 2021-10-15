import argparse
import os.path as path
import sys

from helper_scripts.DataExtractor import DataExtractor

# parser for command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('-v', '--vid', required=True)
parser.add_argument('-i', '--interval', default='30')


def main():
    video_name = args.vid
    frame_interval = int(args.interval)

    if not path.isfile(f'videos/{video_name}.mp4'):
        print('--vid', video_name, "is an invalid video name, make sure video exists in ./videos")
        sys.exit()

    data_extractor = DataExtractor(video_name, frame_interval)
    data_extractor.extract()
    data_extractor.select()
    data_extractor.save()


if __name__ == '__main__':
    args = parser.parse_args()
    main()
