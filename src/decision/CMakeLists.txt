cmake_minimum_required(VERSION 2.8.3)
project(decision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    worldstate
    gate_detect
)

add_definitions(-std=c++14)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES sample_package
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ./include
    ./include/subroutines
    ./include/subroutines/gate
    ../constants
)

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(decision_node
        ../constants/constants.h
        src/decision_node.cpp
        src/DecisionNode.cpp
        include/DecisionNode.h
        src/subroutines/Subroutine.cpp
        include/subroutines/Subroutine.h
        src/subroutines/gate/LocateGate.cpp
        include/subroutines/gate/LocateGate.h
        src/subroutines/gate/LineUpWithGate.cpp
        include/subroutines/gate/LineUpWithGate.h
        src/subroutines/gate/GoThroughGate.cpp
        include/subroutines/gate/GoThroughGate.h)
target_link_libraries(decision_node ${catkin_LIBRARIES})
add_dependencies(decision_node ${catkin_EXPORTED_TARGETS})

add_executable(dead_mans_switch
        src/DeadMansSwitchNode.cpp
        src/dead_mans_switch_node.cpp
        )
target_link_libraries(dead_mans_switch ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if (CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)

    add_rostest_gtest(
            decision_node_rostest
            test/decision_node_test.test
            test/decision_node_rostest.cpp
    )
    target_link_libraries(decision_node_rostest ${catkin_LIBRARIES})

    add_rostest_gtest(
            dead_mans_switch_rostest
            test/dead_mans_switch.test
            test/dead_mans_switch_rostest.cpp
    )
    target_link_libraries(dead_mans_switch_rostest ${catkin_LIBRARIES})
    # TODO: This probably isn't needed, delete
    # Convince CMake this is indeed a C++ project
    #set_target_properties(dead_mans_switch_rostest PROPERTIES LINKER_LANGUAGE CXX)

endif()
