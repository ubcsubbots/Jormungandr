/*
 * Created By: Kevin Huang
 * Created On: June 26, 2020
 * Description: Performs YOLO object detection
 */

#include <YoloDetectorNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "yolo_detect";

    // Create an instance of your class
    YoloDetectorNode filter(argc, argv, node_name);

    // Once the node stops, return 0
    return 0;
}