#ifndef GATE_DETECT_GATEDETECT_H
#define GATE_DETECT_GATEDETECT_H

#include "Gate.h"
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <gate_detect/gateDetectMsg.h>
#include <gate_detect/gatedetectConfig.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class GateDetectNode {
    image_transport::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::string subscribeTopic;
    std::string publishTopic;
    cv::Mat lineImg;
    Gate gate;

  public:
    GateDetectNode(int argc, char** argv, std::string nodeName);

  private:
    void subscriberCallBack(const sensor_msgs::ImageConstPtr& msg);

    void reconfigCallBack(const gate_detect::gatedetectConfig& config,
                          uint32_t level);

    void publishGateDetectMsg(const std::vector<float> gateVector);
};

#endif // GATE_DETECT_GATEDETECT_H