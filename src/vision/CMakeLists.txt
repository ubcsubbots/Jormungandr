cmake_minimum_required(VERSION 2.8.3)
project(vision)

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
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES vision
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
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
add_executable(hsv_filter_node
        src/hsv_filter_node.cpp
        src/HSVFilter.cpp
        src/HSVFilterNode.cpp
        include/HSVFilter.h
        include/HSVFilterNode.h)

## Specify libraries to link a library or executable target against
target_link_libraries(hsv_filter_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hsv_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    catkin_add_gtest(hsv-filter-test test/hsv-filter-test.cpp src/HSVFilter.cpp
            test/ImageTestUtils.h test/ImageTestUtils.cpp
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_link_libraries(hsv-filter-test ${catkin_LIBRARIES} ${OpenCV_LIBS})

    # Adding rostest to the package
    find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
    add_rostest_gtest(hsv_filter_rostest test/hsv_filter_test.test test/hsv_filter_rostest.cpp
            test/ImageTestUtils.h test/ImageTestUtils.cpp)
    target_link_libraries(hsv_filter_rostest ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()
