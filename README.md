# Collecting images for training with Realsense camera and UR robot

## Prerequisite

- install librealsense 2.0
- install python wrapper
- install urx

## How to use

- catkin_make
- To take images
- 
        roslaunch realsense_image_saver image_saver.launch
- To take images with ur robot
-
        roslaunch realsense_image_saver ur_robot.launch
        

- image_saver.py will save images in image folder