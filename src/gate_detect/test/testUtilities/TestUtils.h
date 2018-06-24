/*
 * Created By: Cameron Newton
 * Created On: June 16th, 2018
 * Description: Utilities to aid in debugging GateDetection module
 */

#include "GateDetection.h"

#ifndef PROJECT_TESTUTILS_H
#define PROJECT_TESTUTILS_H

class TestUtils {
  public:
    /**
     * Function to display the gate detected over the inputted image
     * Contains interpolation constants that need to be updated to properly
     * function
     * @param mat
     * @param floatVec
     */
    static void DisplayGateDetected(cv::Mat mat, std::vector<float> floatVec);

    /**
     *
     *
     * @param image
     * @param floatVec
     * @param imgWidth
     * @param imgHeight
     * @return
     */
    static cv::Mat drawGate(cv::Mat image, std::vector<float> floatVec);
};

#endif // PROJECT_TESTUTILS_H
