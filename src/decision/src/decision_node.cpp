/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description: Node responsible for making navigation decisions. Invokes a
 * subroutine for each logical state of
 * operation.
 */
#include "DecisionNode.h"

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "filter";

    // Create an instance of your class
    DecisionNode decisionNode(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}