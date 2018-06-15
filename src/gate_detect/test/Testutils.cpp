#include "TestUtils.h"
#include <iostream>

using namespace std;

void TestUtils::DisplayGateDetected(cv::Mat mat, std::vector<float> floatVec) {
    cv::namedWindow("Display window",
                    cv::WINDOW_AUTOSIZE); // Create a window for display.

    cout << "Gate Vector test gateRightRed %f %f %f %f %f %f %f %f %f",
    floatVec[0], floatVec[1], floatVec[2], floatVec[3], floatVec[4],
    floatVec[5], floatVec[6], floatVec[7], floatVec[8];

    cv::line(mat,
             cv::Point(int(floatVec[0]), 0),
             cv::Point(int(floatVec[0]), mat.rows),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(mat,
             cv::Point(int(floatVec[3]), 0),
             cv::Point(int(floatVec[3]), mat.rows),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(mat,
             cv::Point(0, floatVec[6]),
             cv::Point(mat.cols, floatVec[6]),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);

    cv::imshow("Detected Gate", mat);
    cv::waitKey(0);
}