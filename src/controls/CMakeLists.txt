cmake_minimum_required(VERSION 2.8.3)
project(controls)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_manager
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  rospy
  nav_msgs
  std_msgs
  realtime_tools
  drivers
  localization
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
   INCLUDE_DIRS include/
   CATKIN_DEPENDS  controller_interface hardware_interface pluginlib roscpp
   LIBRARIES controls
#  DEPENDS system_lib
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

add_library(subbots_controllers_lib  
  src/controllers/thruster_array_controller.cpp
 
)

#####################################################################
# BELOW IS AN EXAMPLE PYTHON SCRIPT TO HELP GET SIMULINK_SRC VALUE
######################################################################
## run linux command dir on folder with source files, paste into here
#names = """controller.cpp	      rt_remd_snf.cpp	      xdotc_7IquZDOK.cpp 
#    control_system.cpp    rt_roundd_snf.cpp       xnrm2_qZ7KPglN.cpp
#    cosd_REZfihlV.cpp     sind_X3pRZNku.cpp       xnrm2_UbyzGcEw.cpp
#    frame_conversion.cpp  svd_eAADSaaD.cpp	      xrotg_IPICesPU.cpp
#    invNxN_YgskRBgH.cpp   thrust_allocation.cpp   xrot_XJMNyuag.cpp
#    rtGetInf.cpp	      trajectory_planner.cpp  xscal_myTCeGM2.cpp
#    rtGetNaN.cpp	      xaxpy_7155JtSF.cpp      xswap_cwXDe7Ur.cpp
#    rt_nonfinite.cpp      xaxpy_cpmPaViz.cpp
#    rt_powd_snf.cpp       xaxpy_irZdM5XF.cpp"""
#s = names.split(" ")
#s = list(filter(lambda x: x != "", s))
#s = ["include/simulink/" + x for x in s]
#s = " ".join(s)
## copy the output of this into the catkin var SIMULINK_SRC
#print(s)

set(SIMULINK_SRC 
include/simulink/controller.cpp  include/simulink/rt_remd_snf.cpp        include/simulink/xdotc_7IquZDOK.cpp include/simulink/
 include/simulink/control_system.cpp include/simulink/rt_roundd_snf.cpp include/simulink/xnrm2_qZ7KPglN.cpp                   
 include/simulink/cosd_REZfihlV.cpp include/simulink/sind_X3pRZNku.cpp include/simulink/xnrm2_UbyzGcEw.cpp                    
 include/simulink/frame_conversion.cpp include/simulink/svd_eAADSaaD.cpp                                                      
include/simulink/xrotg_IPICesPU.cpp                                                                                           
 include/simulink/invNxN_YgskRBgH.cpp include/simulink/thrust_allocation.cpp include/simulink/xrot_XJMNyuag.cpp               
 include/simulink/rtGetInf.cpp   include/simulink/trajectory_planner.cpp include/simulink/xscal_myTCeGM2.cpp                  
 include/simulink/rtGetNaN.cpp   include/simulink/xaxpy_7155JtSF.cpp include/simulink/xswap_cwXDe7Ur.cpp                      
 include/simulink/rt_nonfinite.cpp include/simulink/xaxpy_cpmPaViz.cpp                                                        
 include/simulink/rt_powd_snf.cpp include/simulink/xaxpy_irZdM5XF.cpp 
)


add_executable(control_node 
  ${SIMULINK_SRC}
  src/control_node.cpp 
  src/interface/robot_hardware_interface.cpp
)
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(control_node ${catkin_LIBRARIES})
target_link_libraries(subbots_controllers_lib ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test_package.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
catkin_add_gtest(thruster_array_controller_test 
  src/controllers/thruster_array_controller.cpp
  test/thruster_array_controller_test.cpp
)
 

target_link_libraries(thruster_array_controller_test ${catkin_LIBRARIES})
 
