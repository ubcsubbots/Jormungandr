/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Set up environment for GateDetectionNode
 */

#include "GateDetectionNode.h"

GateDetectionNode::GateDetectionNode(int argc, char** argv) {
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it(nh);

    dynamic_reconfigure::Server<gate_detect::gatedetectConfig> server;
    dynamic_reconfigure::Server<gate_detect::gatedetectConfig>::CallbackType f;

    subscribeTopic = "/vision/output";
    publishTopic   = "/gateDetect/output";

    subscriber_ = it.subscribe(
    subscribeTopic, 2, &GateDetectionNode::subscriberCallBack, this);

    gateDetection = GateDetection();

    f = boost::bind(&GateDetectionNode::reconfigCallBack, this, _1, _2);
    server.setCallback(f);

    publisher1_ = nh_.advertise<gate_detect::GateDetectMsg>(publishTopic, 10);
    publisher2_ = it.advertise("gate_image_output", 100);

    ros::spin();
}

void GateDetectionNode::subscriberCallBack(
const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    lineImg = cv_ptr->image;

    std::vector<float> gateVector = gateDetection.initialize(lineImg);

    publishGateDetectMsg(gateVector);

    // Uncomment if you want to view gate seen by node
    // publishGateImage(gateVector);
}

void GateDetectionNode::publishGateDetectMsg(std::vector<float> gateVectorIn) {
    gate_detect::GateDetectMsg msg;

    if (gateVectorIn[0] != 0.0) {
        msg.detectLeft = 1;
    } else {
        msg.detectLeft   = 0;
        msg.angleLeft    = gateVectorIn[1];
        msg.distanceLeft = gateVectorIn[2];
    }
    if (gateVectorIn[3] != 0.0) {
        msg.detectRight = 1;
    } else {
        msg.detectRight   = 0;
        msg.angleRight    = gateVectorIn[4];
        msg.distanceRight = gateVectorIn[5];
    }
    if (gateVectorIn[6] != 0.0) {
        msg.detectTop = 1;
    } else {
        msg.detectTop   = 0;
        msg.angleTop    = gateVectorIn[7];
        msg.distanceTop = gateVectorIn[8];
    }

    publisher1_.publish(msg);
}

void GateDetectionNode::publishGateImage(std::vector<float> gateVector) {
    cv::Mat colourMat;

    cv::cvtColor(lineImg, colourMat, CV_GRAY2BGR);

    colourMat = TestUtils::drawGate(colourMat, gateVector);

    cv_bridge::CvImage out_msg;
    out_msg.header =
    std_msgs::Header(); // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    out_msg.image    = colourMat;                          // Your cv::Mat

    publisher2_.publish(out_msg.toImageMsg());
}

void GateDetectionNode::reconfigCallBack(
const gate_detect::gatedetectConfig& config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %i %i %i %i %i",
             config.cannyLow,
             config.houghLinesThreshold,
             config.houghLinesMinLength,
             config.houghLinesMaxLineGap,
             config.poleMax);

    gateDetection = GateDetection(config.cannyLow,
                                  config.houghLinesThreshold,
                                  config.houghLinesMinLength,
                                  config.houghLinesMaxLineGap,
                                  config.poleMax);
}