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

    dynamic_reconfigure::Server<vision::hsvFilterConfig> server;
    dynamic_reconfigure::Server<vision::hsvFilterConfig>::CallbackType f;

    f = boost::bind(&HSVFilterNode::dynamicreconfigCallback, this, _1, _2);
    server.setCallback(f);

    int hlo, hhi, vlo, vhi, slo, shi;
    nh.getParam("h_low", hlo);
    nh.getParam("h_high", hhi);
    nh.getParam("v_low", vlo);
    nh.getParam("v_high", vhi);
    nh.getParam("s_low", slo);
    nh.getParam("s_high", shi);

    filter_ = HSVFilter(hlo, hhi, slo, shi, vlo, vhi);

    int refresh_rate = 1;

    subscriber_      = it.subscribe(
    subscribeTopic, refresh_rate, &HSVFilterNode::subscriberCallBack, this);

    int queue_size = 1;
    publisher_     = it.advertise(publishTopic, queue_size);

    ros::spin();
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

void HSVFilterNode::dynamicreconfigCallback(
const vision::hsvFilterConfig& config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %i %i %i %i %i %i",
             config.h_low,
             config.s_low,
             config.v_low,
             config.h_high,
             config.s_high,
             config.v_high);

    filter_ = HSVFilter(config.h_low,
                        config.h_high,
                        config.s_low,
                        config.s_high,
                        config.v_low,
                        config.v_high);
}
