/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 */

#ifndef GATE_DETECT_GATEDETECT_H
#define GATE_DETECT_GATEDETECT_H

#include "../test/testUtilities/TestUtils.h"
#include "GateDetection.h"
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <gate_detect/GateDetectMsg.h>
#include <gate_detect/gatedetectConfig.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class GateDetectionNode {
  public:
    GateDetectionNode(int argc, char** argv);

  private:
    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher2_;
    ros::Publisher publisher1_;
    std::string subscribeTopic;
    std::string publishTopic;
    cv::Mat lineImg;
    GateDetection gateDetection;

    /**
     * Image callback function
     *
     * @param msg Image message
     */
    void subscriberCallBack(const sensor_msgs::ImageConstPtr& msg);

    /**
     * Dynamic reconfigure callback function
     *
     * @param config
     * @param level
     */
    void reconfigCallBack(const gate_detect::gatedetectConfig& config,
                          uint32_t level);

    /**
     * Publish gate seen by node
     *
     * @param gateVector vector defining gate seen by node
     */
    void publishGateDetectMsg(const std::vector<float> gateVector);

    /**
     * Publish image of gate seen by node, use for testing
     * @param gateVector
     */
    void publishGateImage(std::vector<float> gateVector);
};

#endif // GATE_DETECT_GATEDETECT_H