#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import rospy
import cv2
import sys
import os

NAME = 'image_saver'
CONTINUE = True  # True for continue to save image where you did last time

def main():
    rospy.init_node(NAME)
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    #image name
    path = '/home/irobot2/catkin_ws/src/realsense_image_saver/image'
    folder_name = 1
    file_name = '00001'
    folder_list = os.listdir(path)
    folder_list.sort()
    rospy.set_param('/save_or_not', False)

    if CONTINUE == False:
        if len(folder_list) == 0:
            folder_name = path + '/first_folder'
            os.mkdir(folder_name)
        else:
            folder_name = path + '/' + folder_list[-1]
            file_list = os.listdir(folder_name)
            file_list.sort()

            if len(file_list) == 0:
                file_name = '00001'
            else:
                file_name = str(int(file_list[-1][:-4]) + 1).zfill(5)
    else:
        if len(folder_list) == 0:
            folder_name = path + '/first_folder'
        else:
            folder_name = path + '/' + folder_list[-1] + '_new'

        os.mkdir(folder_name)


    try:
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            image = np.asanyarray(color_frame.get_data())

            # # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Stack both images horizontally
            # image = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', image)
            cv2.waitKey(1)

            save_it = rospy.get_param('/save_or_not')

            if save_it == True:
                cv2.imwrite(folder_name + '/' + file_name + '.png', image)
                print('saved '+ file_name)
                file_name = str(int(file_name) + 1).zfill(5)
                rospy.set_param('/save_or_not', False)

            rate.sleep()                
                


    finally:
        # Stop streaming
        pipeline.stop()


    
if __name__ == '__main__':
    main()

