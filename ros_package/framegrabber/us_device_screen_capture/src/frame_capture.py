#!/usr/bin/env python3
'''
First Author: Dianye Huang (dianye.huang@tum.de)
Date: 2022-05-05 16:07:48
Description:
    - This scripts will use OpenCV library to capture the video channels indicated
'''

import rospy
import cv2
import yaml
import os

if __name__ == '__main__':

    # ROS initialization
    rospy.init_node('frame_capture_node', anonymous=True)

    # Loading yaml file for the configuration of video streaming
    file_path = os.path.dirname(os.path.dirname(__file__)) + '/config/screen_cap_config.yaml'
    with open(file_path, 'r') as f:
        capture_config = yaml.load(f.read(), Loader=yaml.FullLoader)

    # Frame sizes of the captured screen
    frame_height = capture_config["frame_size"]["height"]
    frame_width = capture_config["frame_size"]["width"]

    # Set the video capture parameters
    screen_cap = cv2.VideoCapture(capture_config["video_index"])
    screen_cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    screen_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    if not screen_cap.isOpened():
        print("Screen is not being captured!!!")

    # Cropped parameters for only the US image on the right side
    # MRI image is discarded and not saved
    x0 = capture_config["frame_cropped_coordinates"]["x0"]
    x1 = capture_config["frame_cropped_coordinates"]["x1"]
    y0 = capture_config["frame_cropped_coordinates"]["y0"]
    y1 = capture_config["frame_cropped_coordinates"]["y1"]

    # 30 frames are saved in each second.
    data_folder_name = r"Image Data"
    main_folder_path = os.path.dirname(os.getcwd())
    data_folder_path = os.path.join(main_folder_path, data_folder_name)

    # Frame rate is 30 frames per second.
    # Captured videos from the surgery have 30 fps, and previous IDP project used 30 fps, as well.
    # Models are also trained with 30 fps from the video recordings, and so that
    # frame rates are consistent with the real case.
    sampling_freq = 30
    r = rospy.Rate(sampling_freq)

    img = None
    while not rospy.is_shutdown():
        r.sleep()

        if not screen_cap.isOpened():
            continue

        # Capture the screen frame
        ret, frame = screen_cap.read()

        if ret is True:
            cropped = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)  # gray image

            # Check for the first time
            if img is None:
                img = cropped
                continue

            if sum(sum(img - cropped)) != 0:
                img = cropped

                # File name has the timestamp of the captured frame image
                ros_time = rospy.get_rostime()
                str_secs = str(ros_time.secs)
                str_nsecs_unfilled = str(ros_time.nsecs)
                str_nsecs = str_nsecs_unfilled.rjust(9, '0')
                str_msecs = str_nsecs[0:3]
                image_filename_png = ''.join([str_secs, '_', str_msecs]) + ".png"

                # Save the frame image
                cv2.imwrite(os.path.join(data_folder_path, image_filename_png), img)

            else:
                pass
        else:
            rospy.loginfo("Not valid!")

    screen_cap.release()


