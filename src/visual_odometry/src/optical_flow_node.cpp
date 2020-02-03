/*
 * Created By: Kevin Huang
 * Created On: February 2nd, 2020
 * Description: Performs visual odometry
 */
#include <OpticalFlowNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "opticalflow";

    // Create an instance of your class
    OpticalFlowNode filter(argc, argv, node_name);

    // Once the node stops, return 0
    return 0;
}