/*
 * Created By: Joel Ahn
 * Created On: March 17th, 2018
 * Description: Run the world state node
 */

#include <WorldStateNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "world_state_node";

    // Create an instance of your class
    WorldStateNode WorldStateNode(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
