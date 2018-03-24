cmake_minimum_required(VERSION 2.8.3)
project(gate_detect)


find_package(catkin REQUIRED COMPONENTS
		cv_bridge
		image_transport
		roscpp
		sensor_msgs
		std_msgs
		rospy
		dynamic_reconfigure
		)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
		cfg/gatedetect.cfg
)

catkin_package(INCLUDE_DIRS
		include
		test
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

enable_testing()

add_executable(gateTest
		test/gateDetectTest.cpp
		include/Gate.h
		src/Gate.cpp
        test/gate.png)

add_dependencies(gate_detect ${PROJECT_NAME}_gencfg)

target_link_libraries(gate_detect
		${catkin_LIBRARIES}
		${OpenCV_LIBS})

target_link_libraries(gateTest
		${catkin_LIBRARIES}
		${OpenCV_LIBS})

