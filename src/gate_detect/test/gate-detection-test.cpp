//
// Created by da-cam on 19/05/18.
//

#include "GateDetection.h"
#include "testUtilities/TestUtils.h"
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

    GateDetection gateDetection;

    std::vector<float> floater = gateDetection.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateDetected(image,floater);
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

    GateDetection gateDetection;

    std::vector<float> floater = gateDetection.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateDetected(image,floater);
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

    GateDetection gateDetection;
    vector<float> floater = gateDetection.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateDetected(image,floater);
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

    GateDetection gateDetection;
    vector<float> floater = gateDetection.initialize(image);

    // Uncomment to view gateDetection drawn over image
    // TestUtils::DisplayGateDetected(image,floater);
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

    GateDetection gateDetection;
    std::vector<float> floater = gateDetection.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateDetected(image,floater);
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

    GateDetection gateDetection;
    std::vector<float> floater = gateDetection.initialize(image);

    // Uncomment to view gate drawn over image
    // TestUtils::DisplayGateDetected(image,floater);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
