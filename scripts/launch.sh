#!/bin/bash

# Source ROS workspace
source ~/catkin_ws/devel/setup.bash

# Launch the ROS node
roslaunch usb_camera_node usb_cam.launch
