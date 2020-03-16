
/*
 * Created By: Logan Fillo
 * Created On: March 15 2020
 * Description: A node that uses dead reckoning using IMU and depth sensor
 *              data to provide a localization estimation
 */

#include <dead_reckoning/DeadReckoningNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    ros::init(argc, argv, "dead_reckoning_node");

    // Create an instance of your class
    DeadReckoningNode dead_reckoning_node(argc, argv);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}