/*
 * Created By: Logan Fillo
 * Created On: March 9th, 2019
 * Description: Run the gate test result node
 */

#include <GateTestResultNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "gate_test_result_node";

    // Create an instance of your class
    GateTestResultNode GateTestResultNode(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
