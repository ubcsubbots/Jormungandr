/*
 * Created By: Cameron Newton
 * Created On: May 5th, 2018
 * Description: Test functionality of LineDetector
 */

#include "test_utilities/line_detect_test_utils.h"
#include <gtest/gtest.h>

using namespace cv;
using namespace std;

TEST(TestSuite, testCase1) {
    Mat image;

    image = imread("test_images/straight_middle.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    LineDetector lineDetector;
    LinesToFollow linesToFollow = lineDetector.initialize(image);

    linesToFollow.frontLine.middleOfMarker

    // TestUtils::drawLineToFollow(image,linesToFollow);
}

TEST(TestSuite, testCase2) {
    Mat image;

    image = imread("test_images/straight_and_right.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    LineDetector lineDetector;
    LinesToFollow linesToFollow = lineDetector.initialize(image);

    // TestUtils::drawLineToFollow(image,linesToFollow);
}

TEST(TestSuite, testCase3) {
    Mat image;

    image = imread("test_images/straight_and_right_offset_left.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    LineDetector lineDetector;
    LinesToFollow linesToFollow = lineDetector.initialize(image);

    // TestUtils::drawLineToFollow(image,linesToFollow);
}

TEST(TestSuite, testCase4) {
    Mat image;

    image = imread("test_images/straight_and_right_offset_right.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    LineDetector lineDetector;
    LinesToFollow linesToFollow = lineDetector.initialize(image);

    // TestUtils::drawLineToFollow(image,linesToFollow);
}

TEST(TestSuite, testCase5) {
    Mat image;

    image = imread("test_images/straight_offset_left.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    LineDetector lineDetector;
    LinesToFollow linesToFollow = lineDetector.initialize(image);

    // TestUtils::drawLineToFollow(image,linesToFollow);
}

TEST(TestSuite, testCase6) {
    Mat image;

    image = imread("test_images/straight_offset_right.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    LineDetector lineDetector;
    LinesToFollow linesToFollow = lineDetector.initialize(image);

    // TestUtils::drawLineToFollow(image,linesToFollow);
}

TEST(TestSuite, testCase7) {
    Mat image;

    image = imread("test_images/turn_left.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    LineDetector lineDetector;
    LinesToFollow linesToFollow = lineDetector.initialize(image);

    // TestUtils::drawLineToFollow(image,linesToFollow);
}

TEST(TestSuite, testCase8) {
    Mat image;

    image = imread("test_images/turn_right.png",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    LineDetector lineDetector;
    LinesToFollow linesToFollow = lineDetector.initialize(image);

    // TestUtils::drawLineToFollow(image,linesToFollow);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
