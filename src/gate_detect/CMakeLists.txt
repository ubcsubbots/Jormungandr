cmake_minimum_required(VERSION 2.8.3)
project(gate_detect)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_definitions(-std=c++14)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  gateDetect.msg
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
   LIBRARIES gate_detect
   CATKIN_DEPENDS message_runtime
)

# Adding gtests to the package
catkin_add_gtest(gate-detect-test test/gate-detect-test.cpp)
target_link_libraries(gate-detect-test ${catkin_LIBRARIES})
add_dependencies(gate-detect-test ${catkin_EXPORTED_TARGETS})
