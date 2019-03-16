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

cv::Mat TestUtils::drawGate(cv::Mat image, GateCoordinates gateCoordinates) {
    int leftPole, rightPole, topPole;

    leftPole = (int) ((image.cols / 2) -
                      ((sin(gateCoordinates.distanceLeftPole) *
                        gateCoordinates.angleLeftPole *
                        ((gateCoordinates.angleLeftPole - 5.134) / -0.002673)) /
                       (.3048 / 4)));

    rightPole = (int) ((
    (image.cols / 2) -
    ((sin(gateCoordinates.distanceRightPole) * gateCoordinates.angleRightPole *
      4.0 * ((gateCoordinates.angleRightPole - 5.134) / -0.002673)) /
     (.3048))));

    topPole = (int) ((image.rows / 2) -
                     ((sin(gateCoordinates.distanceTopPole) *
                       gateCoordinates.angleTopPole * 4.0 *
                       ((gateCoordinates.angleTopPole - 5.134) / -0.002673)) /
                      (.3048)));

    cv::line(image,
             cv::Point(leftPole, 0),
             cv::Point(leftPole, image.rows),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(image,
             cv::Point(rightPole, 0),
             cv::Point(rightPole, image.rows),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(image,
             cv::Point(0, topPole),
             cv::Point(image.cols, topPole),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);

    return image;
}