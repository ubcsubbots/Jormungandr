cmake_minimum_required(VERSION 2.8.3)
project(gate_detect)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
		cv_bridge
		image_transport
		roscpp
		rospy
		dynamic_reconfigure
		message_generation
		)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
		cfg/gatedetect.cfg
)

add_message_files(
		FILES
		gateDetectMsg.msg
)

generate_messages(
		DEPENDENCIES
		std_msgs
)

catkin_package(INCLUDE_DIRS
		include
		LIBRARIES gate_detect

)

catkin_package(
		CATKIN_DEPENDS message_runtime

)

include_directories(
	${catkin_INCLUDE_DIRS}
	${Open_CV_NCLUDE_DIRS}
	./include
)

add_executable(gate_detect
		src/GateDetectNode.cpp
		src/gateDetect.cpp
		src/Gate.cpp
		include/GateDetectNode.h
		include/Gate.h
		)

target_link_libraries(gate_detect
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
)



add_dependencies(gate_detect ${PROJECT_NAME}_gencfg)
add_dependencies(gate_detect ${catkin_EXPORTED_TARGETS})
add_dependencies(gate_detect gate_detect_generate_messages_cpp)

if (CATKIN_ENABLE_TESTING)
	# Adding gtests to the package
	catkin_add_gtest(gateDetectTest test/gateDetectTest.cpp src/Gate.cpp)
	target_link_libraries(gateDetectTest ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()


