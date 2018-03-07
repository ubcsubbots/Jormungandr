/*
 * Created By: Reid Oliveira
 * Created On: January 06, 2018
 * Description:
 */

#include "ImageTestUtils.h"
#include <HSVFilterNode.h>
#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace cv;

/**
 * This is the helper class which will publish and subscribe messages which will
 * test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the
 * subscriber
 *      getter function - to provide a way for gtest to check for equality of
 * the message recieved
 */
class HSVNodeTest : public testing::Test {
  protected:
    virtual void SetUp() {
        image_transport::ImageTransport it =
        image_transport::ImageTransport(nh_);

        std::string publishTopic   = "/camera/image_raw";
        std::string subscribeTopic = "/vision/output";

        test_publisher = it.advertise(publishTopic, 1);
        test_subscriber =
        it.subscribe(subscribeTopic, 1, &HSVNodeTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    cv::Mat image_output;
    image_transport::Publisher test_publisher;
    image_transport::Subscriber test_subscriber;

  public:
    void callback(const sensor_msgs::ImageConstPtr& image) {
        image_output =
        cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8)->image;
    }
};

TEST_F(HSVNodeTest, filterImage) {
    Mat image, expected;
    image = Mat(2, 2, CV_8UC3, Vec3b(0, 0, 0));
    image.at<Vec3b>(Point(0, 0)) = Vec3b(0, 79, 255); // intl orange
    image.at<Vec3b>(Point(0, 1)) = Vec3b(0, 79, 255); // intl orange

    expected = Mat(2, 2, CV_8UC1, Scalar(0));
    image.at<uchar>(Point(0, 0)) = 255;
    image.at<uchar>(Point(0, 1)) = 255;

    cv_bridge::CvImage img;
    img.header   = std_msgs::Header();
    img.encoding = "bgr8";
    img.image    = image;
    test_publisher.publish(img.toImageMsg());

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_TRUE(ImageTestUtils::compareMat(expected, image_output));
}

int main(int argc, char** argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros
    // framework !!
    ros::init(argc, argv, "hsv_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
