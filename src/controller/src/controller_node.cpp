/*
 * Created By: Reid Oliveira
 * Created On: July 22, 2018
 * Description:
 */

#include "ControllerNode.h"

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "controller";

    // Create an instance of your class
    ControllerNode controllerNode(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}