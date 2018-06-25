/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Set up environment for GateDetectorNode
 */

#include "GateDetectorNode.h"

GateDetectorNode::GateDetectorNode(int argc, char** argv) {
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it(nh);

    dynamic_reconfigure::Server<gate_detect::gatedetectConfig> server;
    dynamic_reconfigure::Server<gate_detect::gatedetectConfig>::CallbackType f;

    subscribeTopic = "/vision/output";
    publishTopic   = "/gateDetect/output";

    subscriber_ = it.subscribe(
    subscribeTopic, 2, &GateDetectorNode::subscriberCallBack, this);

    _gateDetector = GateDetector();

    f = boost::bind(&GateDetectorNode::reconfigCallBack, this, _1, _2);
    server.setCallback(f);

    publisher1_ = nh_.advertise<gate_detect::GateDetectMsg>(publishTopic, 10);
    publisher2_ = it.advertise("gate_image_output", 100);

    ros::spin();
}

void GateDetectorNode::subscriberCallBack(
const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    lineImg = cv_ptr->image;

    GateCoordinates gateCoordinates = _gateDetector.initialize(lineImg);

    publishGateDetectMsg(gateCoordinates);

    // Uncomment if you want to view gate seen by node
    // publishGateImage(GateCoordinates);
}

void GateDetectorNode::publishGateDetectMsg(GateCoordinates GateCoordinatesIn) {
    gate_detect::GateDetectMsg msg;

    if (GateCoordinatesIn.detectedLeft == 0.0) {
        msg.detectLeft   = 0;
        msg.angleLeft    = 0;
        msg.distanceLeft = 0;
    } else {
        msg.detectLeft   = 1;
        msg.angleLeft    = GateCoordinatesIn.angleLeft;
        msg.distanceLeft = GateCoordinatesIn.distanceLeft;
    }
    if (GateCoordinatesIn.angleRight == 0.0) {
        msg.detectRight   = 0;
        msg.angleRight    = 0;
        msg.distanceRight = 0;

    } else {
        msg.detectRight   = 1;
        msg.angleRight    = GateCoordinatesIn.angleRight;
        msg.distanceRight = GateCoordinatesIn.distanceRight;
    }
    if (GateCoordinatesIn.detectedTop == 0.0) {
        msg.detectTop   = 0;
        msg.angleTop    = 0;
        msg.distanceTop = 0;

    } else {
        msg.detectTop   = 1;
        msg.angleTop    = GateCoordinatesIn.angleTop;
        msg.distanceTop = GateCoordinatesIn.distanceTop;
    }

    publisher1_.publish(msg);
}

void GateDetectorNode::publishGateImage(std::vector<float> GateCoordinates) {
    cv::Mat colourMat;

    cv::cvtColor(lineImg, colourMat, CV_GRAY2BGR);

    colourMat = TestUtils::drawGate(colourMat, GateCoordinates);

    cv_bridge::CvImage out_msg;
    out_msg.header =
    std_msgs::Header(); // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    out_msg.image    = colourMat;                          // Your cv::Mat

    publisher2_.publish(out_msg.toImageMsg());
}

void GateDetectorNode::reconfigCallBack(
const gate_detect::gatedetectConfig& config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %i %i %i %i %i",
             config.cannyLow,
             config.houghLinesThreshold,
             config.houghLinesMinLength,
             config.houghLinesMaxLineGap,
             config.poleMax);

    _gateDetector = GateDetector(config.cannyLow,
                                 config.houghLinesThreshold,
                                 config.houghLinesMinLength,
                                 config.houghLinesMaxLineGap,
                                 config.poleMax);
}