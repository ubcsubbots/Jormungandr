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
		GateDetectMsg.msg
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
		./test/testUtilities
)

add_executable(gate_detect
        src/GateDetectorNode.cpp
		src/gate_detect_node.cpp
		src/GateDetector.cpp
        include/GateDetectorNode.h
		include/GateDetector.h
        include/Pole.h
        src/Pole.cpp
		test/testUtilities/TestUtils.h
		test/testUtilities/TestUtils.cpp)

target_link_libraries(gate_detect
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
)

add_dependencies(gate_detect ${PROJECT_NAME}_gencfg)
add_dependencies(gate_detect ${catkin_EXPORTED_TARGETS})
add_dependencies(gate_detect gate_detect_generate_messages_cpp)

if (CATKIN_ENABLE_TESTING)
	# Adding gtests to the package
	catkin_add_gtest(gate-detection-test test/gate-detection-test.cpp src/GateDetector.cpp test/testUtilities/TestUtils.cpp src/Pole.cpp
			WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
	target_link_libraries(gate-detection-test ${catkin_LIBRARIES} ${OpenCV_LIBS})

	catkin_add_gtest(gate-detect-msg-test test/gate-detection-msg-test.cpp)
	target_link_libraries(gate-detect-msg-test ${catkin_LIBRARIES})
	add_dependencies(gate-detect-msg-test gate_detect_generate_messages_cpp)
endif()


