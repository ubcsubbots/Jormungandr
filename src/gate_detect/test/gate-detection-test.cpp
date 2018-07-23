/*
 * Created By: Cameron Newton
 * Created On: May 5th, 2018
 * Description: Test functionality of GateDetector
 */

#include "GateDetector.h"
#include "GateTestUtils.h"
#include <gtest/gtest.h>

using namespace cv;
using namespace std;

TEST(TestSuite, testCase1) {
    Mat image;

    image = imread("testImages/gateFrontOrange.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    GateCoordinates gateCoordinates = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateCoordinates(image,floater);
}

TEST(TestSuite, testCase2) {
    Mat image;

    image = imread("testImages/gateFrontRed.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    GateCoordinates gateCoordinates = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateCoordinates(image,floater);
}

TEST(TestSuite, testCase3) {
    Mat image;

    image = imread("testImages/gateLeftOrange.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;
    GateCoordinates gateCoordinates = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateCoordinates(image,floater);
}

TEST(TestSuite, testCase4) {
    Mat image;

    image = imread("testImages/gateLeftRed.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;
    GateCoordinates gateCoordinates = GateDetector.initialize(image);

    // Uncomment to view GateDetector drawn over image
    // TestUtils::DisplayGateCoordinates(image,floater);
}

TEST(TestSuite, testCase5) {
    Mat image;

    image = imread("testImages/gateRightOrange.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;
    GateCoordinates gateCoordinates = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateCoordinates(image,floater);
}

TEST(TestSuite, testCase6) {
    Mat image;

    image =
    imread("testImages/gateRightRed.png", CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;
    GateCoordinates gateCoordinates = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateCoordinates(image,floater);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
