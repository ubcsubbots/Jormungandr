/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 */
#include "GateDetectNode.h"
 
GateDetectNode::GateDetectNode(int argc, char** argv , std::string nodeName) {
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it(nh);

    dynamic_reconfigure::Server<gate_detect::gatedetectConfig> server;
    dynamic_reconfigure::Server<gate_detect::gatedetectConfig>::CallbackType f;

    subscribeTopic = "/uwsim/camera2";
    publishTopic   = "/gateDetect/output";

    subscriber_ = it.subscribe(subscribeTopic, 1, &GateDetectNode::subscriberCallBack, this);

    publisher_     = nh_.advertise<gate_detect::gateDetectMsg>(publishTopic,100);

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

    std::vector<float> gateVector = gate.initialize(cv_ptr->image);

    //publishGateDetectMsg(gateVector);


}

void GateDetectNode::publishGateDetectMsg(std::vector<float> gateVector){

    gate_detect::gateDetectMsg msg;

    if(!(gateVector[0] == 0)) msg.detectLeft = true; msg.angleLeft = gateVector[1]; msg.distanceLeft = gateVector[2];
    if(!(gateVector[3] == 0)) msg.detectRight = true; msg.angleRight = gateVector[4]; msg.distanceRight = gateVector[5];
    if(!(gateVector[6] == 0)) msg.detectTop = true; msg.angleTop = gateVector[7]; msg.distanceTop = gateVector[8];

    publisher_.publish(msg);
}

void GateDetectNode::reconfigCallBack(const gate_detect::gatedetectConfig &config, uint32_t level){


}