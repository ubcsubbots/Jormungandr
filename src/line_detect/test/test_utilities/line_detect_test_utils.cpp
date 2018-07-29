/*
 * Created By: Cameron Newton
 * Created On: June 16th, 2018
 * Description: Utilities to aid in debugging LineDetector module
 */

#include "line_detect_test_utils.h"
#include <iostream>

using namespace cv;

void TestUtils::drawLineToFollow(cv::Mat mat_in, LinesToFollow linesToFollow) {
    line(mat_in,
         cv::Point(linesToFollow.rearLine.middleOfMarker[0],
                   linesToFollow.rearLine.middleOfMarker[1]),
         cv::Point(linesToFollow.rearLine.middleOfMarker[2],
                   linesToFollow.rearLine.middleOfMarker[3]),
         cv::Scalar(0, 255, 0),
         1);
    line(mat_in,
         cv::Point(linesToFollow.frontLine.middleOfMarker[0],
                   linesToFollow.frontLine.middleOfMarker[1]),
         cv::Point(linesToFollow.frontLine.middleOfMarker[2],
                   linesToFollow.frontLine.middleOfMarker[3]),
         cv::Scalar(0, 0, 255),
         1);
    imshow("mat_in", mat_in);
    waitKey(0);
}
