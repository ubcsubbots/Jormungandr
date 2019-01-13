/*
 * Created By: Viral Galaiya
 * Created On: Nov 02, 2018
 * Description:
 */

#include "AccumulatorNode.h"

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "accumulator";

    // Create an instance of your class
    AccumulatorNode accumulator(argc, argv, node_name);

    // Once the node stops, return 0
    return 0;
}
