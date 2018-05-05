cmake_minimum_required(VERSION 2.8.3)
project(worldstate)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  gate_detect
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
  DEPENDENCIES std_msgs
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
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
        ./include
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(world_state_node
    src/WorldStateNode.cpp
    src/world_state_node.cpp
    include/WorldStateNode.h
)

## Specify libraries to link a library or executable target against
target_link_libraries(world_state_node ${catkin_LIBRARIES})

##Possibly remove the first line
add_dependencies(world_state_node ${catkin_EXPORTED_TARGETS})
add_dependencies(world_state_node worldstate_generate_messages_cpp)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hsv_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
#if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
#    catkin_add_gtest(world-state-test
            
#            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#    target_link_libraries(world-state-test ${catkin_LIBRARIES})

    # Adding rostest to the package
#    find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
#    add_rostest_gtest(world_state_rostest
#            )
#    target_link_libraries(world_state_rostest ${catkin_LIBRARIES})
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
