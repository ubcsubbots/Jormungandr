/*
 * Created By: Reid Oliveira
 * Created On: November 11th, 2017
 * Description: Performs an HSV filter on incoming data and republishes it
 */

#include <HSVFilterNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "filter";

    // Create an instance of your class
    HSVFilterNode filter(argc, argv, node_name);

    // Once the node stops, return 0
    return 0;
}