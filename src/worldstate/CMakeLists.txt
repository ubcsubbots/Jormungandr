cmake_minimum_required(VERSION 2.8.3)
project(worldstate)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(cmatkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_definitions(-std=c++14)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  state_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   LIBRARIES worldstate
   CATKIN_DEPENDS message_runtime
)
