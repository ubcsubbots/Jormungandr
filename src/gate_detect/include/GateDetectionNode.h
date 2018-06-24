/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Description: Node that subscribes to image topic and outputs vector defining
 * whether a gate is seen
 */

#ifndef GATE_DETECT_GATEDETECT_H
#define GATE_DETECT_GATEDETECT_H

#include "GateDetection.h"
#include "TestUtils.h"
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
     * @param config reference to config file holding parameters
     * @param level the result of ORing together all of level values of the
     * parameters that have changed
     */
    void reconfigCallBack(const gate_detect::gatedetectConfig& config,
                          uint32_t level);

    /**
     * Publish gate seen by node
     *
     * @param gateVector vector defining gate seen by node
     *
     *          [detectedLeft, distanceLeft, angleLeft, detectedRight,
     *          distanceRight, angleRight, detectedTop, distanceTop, angleTop]
     *
     *          detectedLeft:   0 if left pole not seen, 1 if seen
     *
     *          angleLeft:      Angle from vertical centre to left pole
     *
     *          distanceLeft:   Distance to left pole, 0 if not seen
     *
     *          detectedRight:  0 if right pole not seen, 1 if seen
     *
     *          angleRight:     Angle from right centre to left pole
     *
     *          distanceRight:  Distance to right pole, 0 if not seen
     *
     *          detectedTop:    0 if top pole not seen, 1 if seen
     *
     *          angleTop:       Angle from horizontal centre to top pole
     *
     *          distanceTop:    Distance to top pole, 0 if not seen
     */
    void publishGateDetectMsg(const std::vector<float> gateVector);

    /**
     * Publish image of gate seen by node, use for testing
     * @param gateVector vector containing parameters of gate seen
     *
     *          [detectedLeft, distanceLeft, angleLeft, detectedRight,
     *          distanceRight, angleRight, detectedTop, distanceTop, angleTop]
     *
     *          detectedLeft:   0 if left pole not seen, 1 if seen
     *
     *          angleLeft:      Angle from vertical centre to left pole
     *
     *          distanceLeft:   Distance to left pole, 0 if not seen
     *
     *          detectedRight:  0 if right pole not seen, 1 if seen
     *
     *          angleRight:     Angle from right centre to left pole
     *
     *          distanceRight:  Distance to right pole, 0 if not seen
     *
     *          detectedTop:    0 if top pole not seen, 1 if seen
     *
     *          angleTop:       Angle from horizontal centre to top pole
     *
     *          distanceTop:    Distance to top pole, 0 if not seen
     */
    void publishGateImage(std::vector<float> gateVector);
};

#endif // GATE_DETECT_GATEDETECT_H