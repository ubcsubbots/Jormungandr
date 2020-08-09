cmake_minimum_required(VERSION 2.8.3)
project(yolo_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
        dynamic_reconfigure
        rospy
        message_generation
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
add_message_files(
        FILES
        YoloOut.msg
        YoloOutArray.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
        yolo_detector
)
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES vision
   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ./include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/vision.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(yolo_detector_node
        src/yolo_detector_node.cpp
        src/YoloDetectorNode.cpp
        include/YoloDetectorNode.h)

## Specify libraries to link a library or executable target against
target_link_libraries(yolo_detector_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

add_dependencies(yolo_detector_node yolo_detector_generate_messages_cpp)