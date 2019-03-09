/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Description: Node that subscribes to image topic and outputs vector defining
 * whether a gate is seen
 */

#ifndef GATE_DETECT_GATEDETECT_H
#define GATE_DETECT_GATEDETECT_H

#include "GateDetector.h"
#include "GateTestUtils.h"
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <gate_detect/GateDetectMsg.h>
#include <gate_detect/gatedetectConfig.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class GateDetectorNode {
  public:
    GateDetectorNode(int argc, char** argv);

  private:
    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher2_;
    ros::Publisher publisher1_;
    std::string subscribeTopic;
    std::string publishTopic;
    cv::Mat lineImg;
    GateDetector gateDetector_;
    int width_, height_;
    bool displayDetectedGate_;

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
     * @param GateCoordinates vector defining gate seen by node
     *
     *          [detectedLeftPole, distanceLeftPole, angleLeftPole,
     * detectedRightPole,
     *          distanceRightPole, angleRightPole, detectedTopPole,
     * distanceTopPole, angleTopPole]
     *
     *          detectedLeftPole:   0 if left pole not seen, 1 if seen
     *
     *          angleLeftPole:      Angle from vertical centre to left pole
     *
     *          distanceLeftPole:   Distance to left pole, 0 if not seen
     *
     *          detectedRightPole:  0 if right pole not seen, 1 if seen
     *
     *          angleRightPole:     Angle from right centre to left pole
     *
     *          distanceRightPole:  Distance to right pole, 0 if not seen
     *
     *          detectedTopPole:    0 if top pole not seen, 1 if seen
     *
     *          angleTopPole:       Angle from horizontal centre to top pole
     *
     *          distanceTopPole:    Distance to top pole, 0 if not seen
     */
    void publishGateDetectMsg(const GateCoordinates gateCoordinates);

    /**
     * Publish image of gate seen by node, use for testing
     * @param GateCoordinates vector containing parameters of gate seen
     *
     *          [detectedLeftPole, distanceLeftPole, angleLeftPole,
     * detectedRightPole,
     *          distanceRightPole, angleRightPole, detectedTopPole,
     * distanceTopPole, angleTopPole]
     *
     *          detectedLeftPole:   0 if left pole not seen, 1 if seen
     *
     *          angleLeftPole:      Angle from vertical centre to left pole
     *
     *          distanceLeftPole:   Distance to left pole, 0 if not seen
     *
     *          detectedRightPole:  0 if right pole not seen, 1 if seen
     *
     *          angleRightPole:     Angle from right centre to left pole
     *
     *          distanceRightPole:  Distance to right pole, 0 if not seen
     *
     *          detectedTopPole:    0 if top pole not seen, 1 if seen
     *
     *          angleTopPole:       Angle from horizontal centre to top pole
     *
     *          distanceTopPole:    Distance to top pole, 0 if not seen
     */
    void publishGateImage(GateCoordinates gateCoordinates,cv::Mat image);
};

#endif // GATE_DETECT_GATEDETECT_H