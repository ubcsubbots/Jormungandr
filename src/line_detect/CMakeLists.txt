cmake_minimum_required(VERSION 2.8.3)
project(line_detect)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
		cv_bridge
		image_transport
		roscpp
		rospy
		message_generation
		)

find_package(OpenCV REQUIRED)


add_message_files(
		FILES
		LineDetectMsg.msg
)

generate_messages(
		DEPENDENCIES
		std_msgs
)

catkin_package(INCLUDE_DIRS
		include
		LIBRARIES line_detect
)

catkin_package(
		CATKIN_DEPENDS message_runtime
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${Open_CV_NCLUDE_DIRS}
	./include
)

add_executable(line_detect_node
		src/LineDetectorNode.cpp
        src/line_detect_node.cpp
		src/LineDetector.cpp
		include/LineDetectorNode.h
		include/LineDetector.h
		test/test_utilities/line_detect_test_utils.cpp
		test/test_utilities/line_detect_test_utils.h)

target_link_libraries(line_detect_node
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
)

add_dependencies(line_detect_node ${catkin_EXPORTED_TARGETS})
add_dependencies(line_detect_node gate_detect_generate_messages_cpp)

if (CATKIN_ENABLE_TESTING)
	# Adding gtests to the package
	catkin_add_gtest(line_detect_gtest
			test/line-detection-gtest.cpp
			test/test_utilities/line_detect_test_utils.cpp
			src/LineDetector.cpp
			WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
	target_link_libraries(line_detect_gtest ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()


