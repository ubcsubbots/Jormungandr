/*
 * Created By: Cameron Newton
 * Created On: June 16th, 2018
 * Description: Utilities to aid in debugging GateDetector module
 */

#include "TestUtils.h"
#include <iostream>

using namespace std;

void TestUtils::DisplayGateCoordinates(cv::Mat mat,
                                       std::vector<float> floatVec) {
    cv::namedWindow("Display window",
                    cv::WINDOW_AUTOSIZE); // Create a window for display.

    cout << "Gate Vector test " << floatVec[0] << "  " << floatVec[1] << "  "
         << floatVec[2] << "  " << floatVec[3] << "  " << floatVec[4] << "  "
         << floatVec[5] << "  " << floatVec[6] << "  " << floatVec[7] << "  "
         << floatVec[8] << "\n";

    int leftPole, rightPole, topPole;

    float _interpolationConstant1 = -0.002673, _interpolationConstant2 = 5.134;

    leftPole =
    (mat.cols / 2) -
    ((sin(floatVec[2]) * floatVec[1] *
      ((floatVec[1] - _interpolationConstant2) / _interpolationConstant1)) /
     (.3048 / 4));

    rightPole =
    ((mat.cols / 2) -
     ((sin(floatVec[5]) * floatVec[4] * 4.0 *
       ((floatVec[4] - _interpolationConstant2) / _interpolationConstant1)) /
      (.3048)));

    topPole =
    (int) ((mat.rows / 2) - ((sin(floatVec[8]) * floatVec[7] * 4.0 *
                              ((floatVec[7] - _interpolationConstant2) /
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

cv::Mat TestUtils::drawGate(cv::Mat image, std::vector<float> floatVec) {
    int leftPole, rightPole, topPole;

    leftPole =
    (image.cols / 2) -
    ((sin(floatVec[2]) * floatVec[1] * ((floatVec[1] - 5.134) / -0.002673)) /
     (.3048 / 4));

    rightPole = ((image.cols / 2) - ((sin(floatVec[5]) * floatVec[4] * 4.0 *
                                      ((floatVec[4] - 5.134) / -0.002673)) /
                                     (.3048)));

    topPole = (int) ((image.rows / 2) - ((sin(floatVec[8]) * floatVec[7] * 4.0 *
                                          ((floatVec[7] - 5.134) / -0.002673)) /
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