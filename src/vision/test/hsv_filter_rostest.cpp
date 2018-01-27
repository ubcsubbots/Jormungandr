/*
 * Created By: Reid Oliveira
 * Created On: January 06, 2018
 * Description:
 */

#include <gtest/gtest.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <HSVFilterNode.h>
#include "./ImageTestUtils.h"

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
        it = image_transport::ImageTransport(nh_);
        test_publisher = it.advertise(HSVFilterNode::kSubscribeTopic, 1);
        test_subscriber = it.subscribe(HSVFilterNode::kPublishTopic, 1, &HSVNodeTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it;
    cv::Mat image_output;
    image_transport::Publisher test_publisher;
    image_transport::Subscriber test_subscriber;

public:
    void callback(const sensor_msgs::ImageConstPtr& image) {
        image_output = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
    }
};

TEST_F(HSVNodeTest, filterImage) {
    cv::Mat image, expected;
    image = cv::imread("test_img/test1.png", CV_LOAD_IMAGE_COLOR);
    expected = cv::imread("test_img/result1.png", CV_LOAD_IMAGE_COLOR);

    test_publisher.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg());

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
    ros::init(argc, argv, "my_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
