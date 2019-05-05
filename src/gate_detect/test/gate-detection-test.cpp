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

TEST(TestSuite, front1) {
    Mat image;

    image = imread("testImages/Front1.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, front2) {
    Mat image;

    image = imread("testImages/Front2.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, front3) {
    Mat image;

    image = imread("testImages/Front3.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, left1) {
    Mat image;

    image = imread("testImages/Left1.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, left2) {
    Mat image;

    image = imread("testImages/Left2.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, left3) {
    Mat image;

    image = imread("testImages/Left3.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, left4) {
    Mat image;

    image = imread("testImages/Left4.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, left5) {
    Mat image;

    image = imread("testImages/Left5.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, left6) {
    Mat image;

    image = imread("testImages/Left6.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, Right1) {
    Mat image;

    image = imread("testImages/Right1.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, Right2) {
    Mat image;

    image = imread("testImages/Right1.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, Right3) {
    Mat image;

    image = imread("testImages/Right3.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, Right4) {
    Mat image;

    image = imread("testImages/Right4.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}

TEST(TestSuite, Right5) {
    Mat image;

    image = imread("testImages/Right5.jpg",
                   CV_LOAD_IMAGE_COLOR); // Read the file

    if (image.empty()) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        FAIL();
    }

    GateDetector GateDetector;

    Gate gate = GateDetector.initialize(image);

    // Uncomment to view gate drawn over image
    TestUtils::drawGate(image,gate);
    cv::imshow("image",image);
    cv::waitKey(0);
}



int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
