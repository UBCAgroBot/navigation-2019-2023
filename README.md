# Navigation

> For AgroBot one of the main concern is the robot's navigation. The robot must autonomously navigate the crop rows and for this task we are taking a computer vision approach. The goal of this script is to efficiently detect crop rows on a video.

## Getting started

### How to install

- install [python 3.9](https://www.python.org/downloads/release/python-390/)
- `git clone` this repository
- run `pip install pipenv`
- `cd` into project root
- run `pipenv install` to install project dependencies
- run `pipenv shell` to launch virtual environment

### Scripts

`test_algorithms.py:`
  - runs any of the 4 algorithms against any video
  - arguments :
    - `-a/--alg` : name of algorithm
      - `hough`, `center_row`, `mini_contour` or `scanning`
    - `-v/--vid` : name of video
      - video has to be stored in `./videos/`
      - video configuration has to be stored as a `yaml` at `./config/video/`
    - `-s/--show` : process drawings and show frame
  - example : `python test_algorithms.py -a center_row -v crop`


`create_dataset.py:`
  - aids in data extraction from any video
  - arguments :
    - `-v/--vid` : name of video
      - video has to be stored in `./videos/`
    - `-i/--interval (optional, default = 30)`: interval between keyframes
    - `-o/--overwrite (optional, default = False)` : flag to indicate that you wish to overwrite files at `./extract/vid`
  - example : `python create_dataset.py -v sim` or `python create_dataset.py -v crop -i 60 -o`


`test_pf.py:`
  - runs performance tests on any of the algorithms against any video with an optional GUI
  - reports `framerate`, `time-to-finish`, `vanishing point uptime`, `avg time to process frame`
  - arguments :
    - `-a/--alg` : name of algorithm
      - `hough`, `center_row`, `mini_contour`, `mini_contour_downward`, `scanning`, `seesaw`, or `check_row_end`
    - `-v/--vid` : name of video
      - video has to be stored in `./videos/`
      - video configuration has to be stored as a `yaml` at `./config/video/`
    - `-s/--show` : creates a GUI that shows the frame, allow filter, toggle between views
  - example : `python test_pf.py -a center_row -v crop -s`
  
### Commands to Start World in Gazebo

Run the following Commands the first time:

- source ~/AgroBot/Navigation/agrobot_ws/devel/setup.bash
- In agrobot_ws folder run catkin_make

Run the following commands to open the world:

- Go to agrobot_ws/src folder
- Run bash script (start_corn_fields.sh or start_wheat_fields.sh)

To run PID, open a new terminal:

- Run run_controller bash script

To reset the position of the robot:

- Edit > Reset model poses (to reset the position of your robot to the start position)

## Demo

![](/readme_files/demo_vid.mp4)

### Colour Filtering

To accomplish this task, we need to first denoise the image. In particular, since we know we are looking for crops which
are green in colour, we can filter based on colour. We convert our image to HSV format and define upper and lower bound
for shades of green we capture. For example, here is an original frame from the video:

![crop image](/readme_files/crop.png)

Now, we use the upper and lower bounds on our green colour to generate a mask. This is the resulting mask:

![crop mask](/readme_files/mask.png)

By using bitwise and operator we see that this mask does correspond to the green regions of our image:

![crop bitwise](/readme_files/greenregions.png)

### Denoising and Smoothing

Next, we need to denoise the resulting mask (which is a binary image containing the crop rows). To do this, we perform
gaussian blurring following by multiple iterations of dilation. Here is the resulting mask after dilation:

![denoising](/readme_files/denoising.png)

The reason we use dilation is because we want to fill the gaps in the mask to get a cleaner representation of our rows
and also we want rows that are far from the camera to be merged into one. This is because these rows are not relevant to
our navigation system and by merging them, we can avoid detecting them in our edge and line detection in subsequent
steps.

### Line Detection

Although, the mask image is a binary image and it can be used directly with Hough transform, the large number of points
in the image lead to noisy and slow computation. Hence, Canny edge detection was used to first decrease the number of
points in image. The following image shows the results of the edge detection:

![line detection](/readme_files/linedetection.png)

Following this, we used Hough lines transform. We used the probabilistic version implemented in openCV due to faster
compute time and also the ability to specify parameters such as minimum line length and maximum line gap. The following
image shows the resulting lines we detected.

![Houghlines](/readme_files/Houghlines.png)

### How does this fit in with the rest of the system?

We are currently using these lines and calculating their intersection which occurs at vanishing point. Then we are using
a PID controller to minimimize the distance of this vanishing point from the center of our frame. Using this method, we
are able to centre our chassis over the crop rows we are traversing.
