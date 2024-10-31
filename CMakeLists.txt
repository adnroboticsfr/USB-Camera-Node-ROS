cmake_minimum_required(VERSION 3.0.2)
project(usb_camera_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/usb_cam_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
