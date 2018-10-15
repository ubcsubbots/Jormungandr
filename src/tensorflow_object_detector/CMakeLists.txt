cmake_minimum_required(VERSION 2.8.3)
project(tensorflow_object_detector)
include(ExternalData)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
)

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/detect_ros.py 
			DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)
