//
// Created by da-cam on 19/05/18.
//

// Bring in my package's API, which is what I'm testing
#include "Gate.h"
// Bring in gtest
#include <gtest/gtest.h>

#include "Testutils.cpp"

using namespace cv;
using namespace std;

// Declare a test
TEST(TestSuite, testCase1) {
    cv::Mat image;

    image =
    imread("/home/da-cam/Jormungandr/src/gate_detect/test/gateFrontOrange.png",
           cv::IMREAD_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
    }

    TestUtils::

    Gate gate;

    std::vector<float> floater = gate.initialize(image);

    EXPECT_TRUE(true);
}

// Declare a test
TEST(TestSuite, testCase2) {
    cv::Mat image;

    image =
    imread("/home/da-cam/Jormungandr/src/gate_detect/test/gateFrontRed.png",
           cv::IMREAD_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
    }

    Gate gate;
    std::vector<float> floater = gate.initialize(image);

    EXPECT_TRUE(true);
}

// Declare a test
TEST(TestSuite, testCase3) {
    cv::Mat image;

    image =
    imread("/home/da-cam/Jormungandr/src/gate_detect/test/gateLeftOrange.png",
           cv::IMREAD_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
    }

    Gate gate;
    vector<float> floater = gate.initialize(image);

    EXPECT_TRUE(true);
}

// Declare a test
TEST(TestSuite, testCase4) {
    cv::Mat image;

    image =
    imread("/home/da-cam/Jormungandr/src/gate_detect/test/gateLeftRed.png",
           cv::IMREAD_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
    }

    Gate gate;
    vector<float> floater = gate.initialize(image);

    EXPECT_TRUE(true);
}

// Declare a test
TEST(TestSuite, testCase5) {
    cv::Mat image;

    image =
    imread("/home/da-cam/Jormungandr/src/gate_detect/test/gateRightOrange.png",
           cv::IMREAD_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
    }

    Gate gate;
    std::vector<float> floater = gate.initialize(image);

    EXPECT_TRUE(true);
}

// Declare a test
TEST(TestSuite, testCase6) {
    cv::Mat image;

    image =
    imread("/home/da-cam/Jormungandr/src/gate_detect/test/gateRightRed.png",
           cv::IMREAD_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
    }

    Gate gate;
    std::vector<float> floater = gate.initialize(image);

    /*
       cv::namedWindow("Display window",
                    cv::WINDOW_AUTOSIZE); // Create a window for display.

        ROS_INFO("Gate Vector test gateRightRed %f %f %f %f %f %f %f %f %f",
                 floater[0],
                 floater[1],
                 floater[2],
                 floater[3],
                 floater[4],
                 floater[5],
                 floater[6],
                 floater[7],
                 floater[8]);


    cv::line(image,
             cv::Point(int(floater[0]), 0),
             cv::Point(int(floater[0]), image.rows),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(image,
             cv::Point(int(floater[3]), 0),
             cv::Point(int(floater[3]), image.rows),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);
    cv::line(image,
             cv::Point(0, floater[6]),
             cv::Point(image.cols, floater[6]),
             cv::Scalar(0, 0, 255),
             3,
             CV_AA);

     cv::imshow("New gateRightRed", image);
     cv::waitKey(0);
     */

    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

/*
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window
   for display.
    cv::imshow( "Display window", image );                   // Show our image
   inside it.
    cv::waitKey(0);
 */
