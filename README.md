# Commands to Start World in Gazebo

Run the following Commands the first time:
-	source ~/AgroBot/Navigation/agrobot_ws/devel/setup.bash
-	In agrobot_ws folder run catkin_make

Run the following commands to open the world:
-	Go to agrobot_ws/src folder
-	Run bash script (start_corn_fields.sh or start_wheat_fields.sh)

To run PID, open a new terminal:
- Run run_controller bash script

To reset the position of the robot:
-	Edit > Reset model poses (to reset the position of your robot to the start position)


# Navigation
<table>
<tr>
<td>
 For AgroBot one of the main concern is the robot's navigation. The robot must autonomously navigate the crop rows and for this task we are taking a computer vision approach. The goal of this script is to efficiently detect crop rows on a video.
</td>
</tr>
</table>


## Demo
Here is a working live demo :   ![](/Demo/Demo_2.mp4)


### Colour Filtering
To accomplish this task, we need to first denoise the image. In particular, since we know we are looking for crops which are green in colour, we can filter based on colour. We convert our image to HSV format and define upper and lower bound for shades of green we capture. For example, here is an original frame from the video:

![](/Demo/crop.png)

Now, we use the upper and lower bounds on our green colour to generate a mask. This is the resulting mask:

![](/Demo/mask.png)

By using bitwise and operator we see that this mask does correspond to the green regions of our image:

![](/Demo/greenregions.png)


## Denoising and Smoothing
Next, we need to denoise the resulting mask (which is a binary image containing the crop rows). To do this, we perform gaussian blurring following by multiple iterations of dilation. Here is the resulting mask after dilation:

![](/Demo/denoising.png)

The reason we use dilation is because we want to fill the gaps in the mask to get a cleaner representation of our rows and also we want rows that are far from the camera to be merged into one. This is because these rows are not relevant to our navigation system and by merging them, we can avoid detecting them in our edge and line detection in subsequent steps.


### Line Detection
Although, the mask image is a binary image and it can be used directly with Hough transform, the large number of points in the image lead to noisy and slow computation. Hence, Canny edge detection was used to first decrease the number of points in image. The following image shows the results of the edge detection:

![](/Demo/linedetection.png)

Following this, we used Hough lines transform. We used the probabilistic version implemented in openCV due to faster compute time and also the ability to specify parameters such as minimum line length and maximum line gap. The following image shows the resulting lines we detected.

![](/Demo/Houghlines.png)

### How does this fit in with the rest of the system?
We are currently using these lines and calculating their intersection which occurs at vanishing point. Then we are using a PID controller to minimimize the distance of this vanishing point from the center of our frame. Using this method, we are able to centre our chassis over the crop rows we are traversing.


