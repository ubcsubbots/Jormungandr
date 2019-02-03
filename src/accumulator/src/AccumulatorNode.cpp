/*
 * Created By: Viral Galaiya
 * Created On: Nov 02, 2018
 * Description:
 */

#include "AccumulatorNode.h"

AccumulatorNode::AccumulatorNode(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);  // Ros function for subscribing to and publishing images
    std::string publishTopic   = "/vision/output";
    std::string subscribeTopic = "/camera/HSV_image";

    accumulator_ = Accumulator();

    int refresh_rate = 2;
    subscriber_ = it.subscribe(subscribeTopic, refresh_rate, &AccumulatorNode::subscriberCallBack, this);

    int queue_size = 10;
    publisher_ = it.advertise(publishTopic, queue_size);

    ros::spin();
}

void AccumulatorNode::subscriberCallBack(const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

        accumulator_.mask(cv_ptr->image);
        publishMask(accumulator_.acc);

}

void AccumulatorNode::publishMask(const cv::Mat& mask) {
    publisher_.publish(
            cv_bridge::CvImage(std_msgs::Header(), "mono8", mask)
                    .toImageMsg());
}
