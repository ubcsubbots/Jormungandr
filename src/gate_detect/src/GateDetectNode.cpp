/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 */

#include "GateDetectNode.h"

GateDetectNode::GateDetectNode(int argc, char** argv, std::string nodeName) {
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it(nh);

    dynamic_reconfigure::Server<gate_detect::gatedetectConfig> server;
    dynamic_reconfigure::Server<gate_detect::gatedetectConfig>::CallbackType f;

    subscribeTopic = "/vision/output";
    publishTopic   = "/gateDetect/output";

    subscriber_ =
    it.subscribe(subscribeTopic, 1, &GateDetectNode::subscriberCallBack, this);

    publisher_ = nh_.advertise<gate_detect::gateDetectMsg>(publishTopic, 100);

    f = boost::bind(&GateDetectNode::reconfigCallBack, this, _1, _2);
    server.setCallback(f);

    ros::spin();
}

void GateDetectNode::subscriberCallBack(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (!gate.checkMat()) { gate = Gate(); }

    std::vector<float> gateVector = gate.initialize(cv_ptr->image);

    publishGateDetectMsg(gateVector);
}

void GateDetectNode::publishGateDetectMsg(std::vector<float> gateVectorIn) {
    gate_detect::gateDetectMsg msg;

    if (gateVectorIn[0] != 0.0)
        msg.detectLeft = 1;
    else
        msg.detectLeft = 0;
    msg.angleLeft      = gateVectorIn[1];
    msg.distanceLeft   = gateVectorIn[2];
    if (gateVectorIn[3] != 0.0)
        msg.detectRight = 1;
    else
        msg.detectRight = 0;
    msg.angleRight      = gateVectorIn[4];
    msg.distanceRight   = gateVectorIn[5];
    if (gateVectorIn[6] != 0.0)
        msg.detectTop = 1;
    else
        msg.detectTop = 0;
    msg.angleTop      = gateVectorIn[7];
    msg.distanceTop   = gateVectorIn[8];

    publisher_.publish(msg);
}

void GateDetectNode::reconfigCallBack(
const gate_detect::gatedetectConfig& config, uint32_t level) {}