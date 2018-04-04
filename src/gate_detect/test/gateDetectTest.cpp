// Bring in my package's API, which is what I'm testing
#include "Gate.h"
// Bring in gtest
#include <gtest/gtest.h>

using namespace cv;
using namespace std;

// Declare a test
TEST(TestSuite, testCase1)
{
    cv::Mat image;

    image = imread("/home/da-cam/Jormungandr/src/gate_detect/test/gateLS.png", cv::IMREAD_COLOR); // Read the file

    if(image.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
    }

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", image );                   // Show our image inside it.
    cv::waitKey(0);

    Gate gate;
    std::vector<float> floater= gate.initialize(image);
    for (float gatecoord : floater) cout << gatecoord;

   EXPECT_TRUE(true);
}

// Declare a test
TEST(TestSuite, testCase2)
{
    cv::Mat image;

    image = imread("/home/da-cam/Jormungandr/src/gate_detect/test/3sidedgate.png", cv::IMREAD_COLOR); // Read the file
    if(image.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
    }

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", image );                   // Show our image inside it.
    cv::waitKey(0);

    Gate gate;
    std::vector<float> floater= gate.initialize(image);

    EXPECT_TRUE(true);
}

// Declare a test
TEST(TestSuite, testCase3)
{
    cv::Mat image;

    image = imread("/home/da-cam/Jormungandr/src/gate_detect/test/gateRS.png", cv::IMREAD_COLOR); // Read the file
    if(image.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
    }

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", image );                   // Show our image inside it.
    cv::waitKey(0);

    Gate gate;
    std::vector<float> floater= gate.initialize(image);

    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;


    return RUN_ALL_TESTS();
}
