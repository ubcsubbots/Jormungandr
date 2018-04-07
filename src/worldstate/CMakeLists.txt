cmake_minimum_required(VERSION 2.8.3)
project(worldstate)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_definitions(-std=c++14)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

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
    catkin_add_gtest(world-state-msg-test test/world_state_msg_test.cpp)
    target_link_libraries(world-state-msg-test ${catkin_LIBRARIES})
endif()

add_dependencies(world-state-msg-test ${catkin_EXPORTED_TARGETS})
add_dependencies(world-state-msg-test world_state_msg_test_generate_messages_cpp)

