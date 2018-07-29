/*
 * Created By: Cameron Newton
 * Created On: July 17, 2018
 * Initialize ros node
 */

#include "LineDetectorNode.h"

using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_detect_node");

    LineDetectorNode lineDetectorNode = LineDetectorNode(argc, argv);

    return 0;
}
