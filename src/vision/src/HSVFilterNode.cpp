/*
 * Created By: Reid Oliveira
 * Created On: November 11th, 2017
 * Description: Performs an HSV filter on incoming data and republishes it
 */

#include <HSVFilterNode.h>

HSVFilterNode::HSVFilterNode(int argc, char** argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    image_transport::ImageTransport it(nh);

    std::string subscribeTopic = "/camera/image_raw";
    std::string publishTopic   = "/vision/output";

    XmlRpc::XmlRpcValue hsv;
    if (!private_nh.getParam("hsv", hsv)) {
        ROS_INFO_STREAM(nh.getNamespace()
                        << ": no value given for hsv, using default values");
        filter_ = HSVFilter();
    } else {
        filter_ = HSVFilter(hsv["h_low"],
                            hsv["h_high"],
                            hsv["s_low"],
                            hsv["s_high"],
                            hsv["v_low"],
                            hsv["v_high"]);
    }

    int refresh_rate = 1;
    subscriber_      = it.subscribe(
    subscribeTopic, refresh_rate, &HSVFilterNode::subscriberCallBack, this);

    int queue_size = 1;
    publisher_     = it.advertise(publishTopic, queue_size);
}

void HSVFilterNode::subscriberCallBack(
const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat filtered;
    filter_.apply(cv_ptr->image, filtered);
    publishFilteredImage(filtered);
}

void HSVFilterNode::publishFilteredImage(const cv::Mat& filtered_image) {
    publisher_.publish(
    cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered_image)
    .toImageMsg());
}