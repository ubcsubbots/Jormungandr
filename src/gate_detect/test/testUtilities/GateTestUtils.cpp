/*
 * Created By: Cameron Newton
 * Created On: June 16th, 2018
 * Description: Utilities to aid in debugging GateDetector module
 */

#include "GateTestUtils.h"
#include <iostream>

using namespace std;

void TestUtils::DisplayGateCoordinates(cv::Mat mat,
                                       GateCoordinates gateCoordinates) {
    cv::namedWindow("Display window",
                    cv::WINDOW_AUTOSIZE); // Create a window for display.

    cout << "Gate Vector test " << gateCoordinates.detectedLeftPole << "  "
         << gateCoordinates.angleLeftPole << "  "
         << gateCoordinates.distanceLeftPole << "  "
         << gateCoordinates.detectedRightPole << "  "
         << gateCoordinates.angleRightPole << "  "
         << gateCoordinates.distanceRightPole << "  "
         << gateCoordinates.detectedTopPole << "  "
         << gateCoordinates.angleTopPole << "  "
         << gateCoordinates.distanceTopPole << "\n";

    int leftPole, rightPole, topPole;

    float _interpolationConstant1 = 81.88,
          _interpolationConstant2 = (float) -.92791;

    leftPole =
    (int) ((mat.cols / 2) -
           ((sin(gateCoordinates.distanceLeftPole) *
             gateCoordinates.angleLeftPole *
             ((gateCoordinates.angleLeftPole - _interpolationConstant2) /
              _interpolationConstant1)) /
            (.3048 / 4)));

    rightPole = (int) ((
    (mat.cols / 2) -
    ((sin(gateCoordinates.distanceRightPole) * gateCoordinates.angleRightPole *
      4.0 * ((gateCoordinates.angleRightPole - _interpolationConstant2) /
             _interpolationConstant1)) /
     (.3048))));

    cv::Mat mat1;

    topPole =
    (int) ((mat.rows / 2) -
           ((sin(gateCoordinates.distanceTopPole) *
             gateCoordinates.angleTopPole * 4.0 *
             ((gateCoordinates.angleTopPole - _interpolationConstant2) /
              _interpolationConstant1)) /
            (.3048)));

    cv::line(mat,
             cv::Point(leftPole, 0),
             cv::Point(leftPole, mat.rows),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(mat,
             cv::Point(rightPole, 0),
             cv::Point(rightPole, mat.rows),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(mat,
             cv::Point(0, topPole),
             cv::Point(mat.cols, topPole),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);

    cv::imshow("Detected Gate", mat);
    cv::waitKey(0);
}

cv::Mat TestUtils::drawGate(cv::Mat image, Gate gate) {
    cv::line(image,//draws the left pole on the image
             cv::Point(gate.leftPole.getMiddleLine()[0],
                       gate.leftPole.getMiddleLine()[1]),
             cv::Point(gate.leftPole.getMiddleLine()[2],
                       gate.leftPole.getMiddleLine()[3]),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(image,//draws the right pole on the image
             cv::Point(gate.rightPole.getMiddleLine()[0],
                       gate.rightPole.getMiddleLine()[1]),
             cv::Point(gate.rightPole.getMiddleLine()[2],
                       gate.rightPole.getMiddleLine()[3]),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(
    image,//draws the top pole on the image
    cv::Point(gate.topPole.getMiddleLine()[0], gate.topPole.getMiddleLine()[1]),
    cv::Point(gate.topPole.getMiddleLine()[2], gate.topPole.getMiddleLine()[3]),
    cv::Scalar(0, 0, 255),
    3,
    CV_AA);

    return image;
}