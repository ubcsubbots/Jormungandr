//
// Created by da-cam on 28/02/18.
//

#include "GateDetectionNode.h"

using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "gate_detect_node");

    GateDetectionNode gateDetect = GateDetectionNode(argc, argv);

    return 0;
}
