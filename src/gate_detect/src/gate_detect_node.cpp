/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Description: Initialize ros node
 */

#include "GateDetectorNode.h"

using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "gate_detect_node");

    GateDetectorNode gateDetect = GateDetectorNode(argc, argv);

    return 0;
}
