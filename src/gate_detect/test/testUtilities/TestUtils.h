//
// Created by da-cam on 15/06/18.
//

#include "GateDetection.h"

#ifndef PROJECT_TESTUTILS_H
#define PROJECT_TESTUTILS_H

class TestUtils {
  public:
    static void DisplayGateDetected(cv::Mat mat, std::vector<float> floatVec);

    static cv::Mat drawGate(cv::Mat image,
                            std::vector<float> floatVec,
                            int imgWidth,
                            int imgHeight);
};

#endif // PROJECT_TESTUTILS_H
