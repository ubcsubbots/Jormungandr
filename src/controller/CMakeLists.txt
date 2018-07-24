cmake_minimum_required(VERSION 2.8.3)
project(controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED cmake_modules)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        )

find_package(Eigen REQUIRED)

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
    DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ./include
)

## Declare a C++ library
# add_library(sample_package
#   src/${PROJECT_NAME}/sample_package.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(sample_package ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(controller_node
        ./include/Controller.h
        ./src/Controller.cpp
        ./include/ControllerNode.h
        ./src/ControllerNode.cpp
        ./src/controller_node.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(controller_node ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    #catkin_add_gtest(my-node-test test/my-node-test.cpp src/MyNode.cpp)
    #target_link_libraries(my-node-test ${catkin_LIBRARIES})

    # Adding rostest to the package
    find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
  #  add_rostest_gtest(my_node_rostest test/sample_package_test.test test/my_node_rostest.cpp)
    #target_link_libraries(my_node_rostest ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
