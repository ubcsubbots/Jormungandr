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

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        ./include
)

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
#if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
#    catkin_add_gtest(world_state_test src/WorldStateNode.cpp)
#    target_link_libraries(world_state_test ${catkin_LIBRARIES})
            
    #WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    #target_link_libraries(world-state-test ${catkin_LIBRARIES})

    # Adding rostest to the package
#    find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
#    add_rostest_gtest(
#            world_state_node_rostest
#            test/world_state_node_rostest.cpp
#            test/world_state_node_test.test)
#    target_link_libraries(world_state_node_rostest ${catkin_LIBRARIES})
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    catkin_add_gtest(world_state_msg_test test/world_state_msg_test.cpp)
    target_link_libraries(world_state_msg_test ${catkin_LIBRARIES})
endif()

