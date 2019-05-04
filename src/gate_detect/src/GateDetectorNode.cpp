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
    publishTopic   = "/gate_detect/output";

    int cannyLow, houghLinesThreshold, houghLinesMinLength, poleMax,
    lowVertThresh, lowHorThresh, houghLinesMaxLineGap;
    double interpolationConstant1, interpolationConstant2;

    // Refer to Gate_Detector.h for parameter descriptions
    nh.getParam("/gate_detect_node/cannyLow", cannyLow);
    nh.getParam("/gate_detect_node/houghLinesThreshold", houghLinesThreshold);
    nh.getParam("/gate_detect_node/houghLinesMinLength", houghLinesMinLength);
    nh.getParam("/gate_detect_node/houghLinesMaxLineGap", houghLinesMaxLineGap);
    nh.getParam("/gate_detect_node/poleMax", poleMax);
    nh.getParam("/gate_detect_node/lowVertThresh", lowVertThresh);
    nh.getParam("/gate_detect_node/lowVertThresh", lowHorThresh);
    nh.getParam("/gate_detect_node/interpolationConstant1",
                interpolationConstant1);
    nh.getParam("/gate_detect_node/interpolationConstant2",
                interpolationConstant2);
    nh.getParam("/gate_detect_node/displayDetectedGate", displayDetectedGate_);

    gateDetector_ = GateDetector(cannyLow,
                                 houghLinesThreshold,
                                 houghLinesMinLength,
                                 houghLinesMaxLineGap,
                                 poleMax,
                                 lowVertThresh,
                                 lowHorThresh);

    interpolator_ =
    Interpolator(interpolationConstant1, interpolationConstant2);

    subscriber_ = it.subscribe(
    subscribeTopic, 2, &GateDetectorNode::subscriberCallBack, this);

    f = boost::bind(&GateDetectorNode::reconfigCallBack, this, _1, _2);
    server.setCallback(f);

    gate_msg_publisher_ =
    nh_.advertise<gate_detect::GateDetectMsg>(publishTopic, 10);
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

    cv::Mat image = (cv_ptr->image);

    Gate gate = gateDetector_.initialize(
    image); // outputs structure of lines which compose the gate
    if (displayDetectedGate_) // debug image containing detected lines is output
                              // here if parameter is true
        publishGateImage(gate, image);

    // these functions interpolate distance/angles
    GateCoordinates gateCoordinates =
    defaultGateCoordinates(); // initializes everything to zeroes

    if (gate.leftDetected) {
        gateCoordinates.detectedLeftPole = true;
        int leftWidth                    = gate.leftPole.getVertWidth();
        gateCoordinates.distanceLeftPole =
        interpolator_.getVertDistance(leftWidth);
        gateCoordinates.angleLeftPole =
        interpolator_.getVertAngle(gate.leftPole.getVertMid(),
                                   leftWidth,
                                   image.cols,
                                   gateCoordinates.distanceLeftPole);
    }
    if (gate.rightDetected) {
        gateCoordinates.detectedRightPole = true;
        int rightWidth                    = gate.rightPole.getVertWidth();
        gateCoordinates.distanceRightPole =
        interpolator_.getVertDistance(rightWidth);
        gateCoordinates.angleRightPole =
        interpolator_.getVertAngle(gate.rightPole.getVertMid(),
                                   rightWidth,
                                   image.cols,
                                   gateCoordinates.distanceRightPole);
    }
    if (gate.topDetected) {
        gateCoordinates.detectedTopPole = true;
        int topWidth                    = gate.topPole.getHorWidth();
        gateCoordinates.distanceTopPole =
        interpolator_.getHorDistance(topWidth);
        gateCoordinates.angleTopPole =
        interpolator_.getHorAngle(gate.topPole.getHorMid(),
                                  topWidth,
                                  image.rows,
                                  gateCoordinates.distanceTopPole);
    }
    publishGateDetectMsg(gateCoordinates);
}

void GateDetectorNode::publishGateDetectMsg(GateCoordinates gateCoordinates) {
    gate_detect::GateDetectMsg msg;

    msg.detectedLeftPole = gateCoordinates.detectedLeftPole;
    msg.angleLeftPole    = gateCoordinates.angleLeftPole;
    msg.distanceLeftPole = gateCoordinates.distanceLeftPole;

    msg.detectedRightPole = gateCoordinates.detectedRightPole;
    msg.angleRightPole    = gateCoordinates.angleRightPole;
    msg.distanceRightPole = gateCoordinates.distanceRightPole;

    msg.detectedTopPole = gateCoordinates.detectedTopPole;
    msg.angleTopPole    = gateCoordinates.angleTopPole;
    msg.distanceTopPole = gateCoordinates.distanceTopPole;

    gate_msg_publisher_.publish(msg);
}

void GateDetectorNode::publishGateImage(Gate gate, cv::Mat image) {
    cv::Mat colourMat;

    cv::cvtColor(
    image,
    colourMat,
    CV_GRAY2BGR); // converts to BGR so colors can be drawn on image

    colourMat =
    TestUtils::drawGate(colourMat, gate); // draws detected lines on image

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

    gateDetector_.setParams(config.cannyLow,
                            config.houghLinesThreshold,
                            config.houghLinesMinLength,
                            config.houghLinesMaxLineGap,
                            config.poleMax,
                            config.lowVertThresh,
                            config.lowHorThresh);
}
