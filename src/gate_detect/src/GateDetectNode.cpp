/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 */
#include "GateDetectNode.h"
 
GateDetectNode::GateDetectNode(int argc, char** argv , std::string nodeName) {
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    dynamic_reconfigure::Server<gate_detect::gatedetectConfig> server;
    dynamic_reconfigure::Server<gate_detect::gatedetectConfig>::CallbackType f;

    subscribeTopic = "/uwsim/camera2";
    publishTopic   = "/gateDetect/output";

    subscriber_ = it.subscribe(subscribeTopic, 1, &GateDetectNode::subscriberCallBack, this);

    publisher_     = it.advertise(publishTopic, 1);

    f = boost::bind(&GateDetectNode::reconfigCallBack, this, _1, _2);
    server.setCallback(f);

    ros::spin();
}

void GateDetectNode::subscriberCallBack(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (!gate.checkMat()){
        gate = Gate();
    }

    gate.initialize(cv_ptr->image);

}

void GateDetectNode::publishOutputImage(cv::Mat mat){

    publisher_.publish(
            cv_bridge::CvImage(std_msgs::Header(), "mono8", mat)
                    .toImageMsg());

}

void GateDetectNode::reconfigCallBack(const gate_detect::gatedetectConfig &config, uint32_t level){


}