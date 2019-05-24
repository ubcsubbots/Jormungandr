/*
 * Created By: Cameron Newton
 * Created On: June 16th, 2018//
 * Description: Utilities to aid in debugging GateDetector module
 */

#include "../../include/GateDetector.h"

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
    static void DisplayGateCoordinates(cv::Mat mat,
                                       GateCoordinates gateCoordinates);

    /**
     *
     *
     * @param image
     * @param floatVec
     * @param imgWidth
     * @param imgHeight
     * @return
     */
    static cv::Mat drawGate(cv::Mat image, Gate gate);
};

#endif // PROJECT_TESTUTILS_H
