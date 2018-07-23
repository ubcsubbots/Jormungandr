/*
 * Created By: Cameron Newton
 * Created On: July 17, 2018
 * Set up environment for LineDetectorNode
 * Description: Node that subscribes to image topic and outputs vector defining
 * whether a line is seen
 */

#ifndef LINE_DETECT_LINEDETECT_H
#define LINE_DETECT_LINEDETECT_H

#include "LineDetector.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <line_detect/LineDetectMsg.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class LineDetectorNode {
  public:
    LineDetectorNode(int argc, char** argv);

  private:
    image_transport::Subscriber subscriber_;
    ros::Publisher publisher1_;
    std::string subscribeTopic_;
    std::string publishTopic_;
    cv::Mat lineImg;
    LineDetector lineDetector_;

    /**
     * Image callback function
     *
     * @param msg Image message
     */
    void subscriberCallBack(const sensor_msgs::ImageConstPtr& msg);

    /**
     * Publish Line seen by node
     *
     */
    void publishLineDetectMsg(const LinesToFollow linesToFollow);
};

#endif // LINE_DETECT_LINEDETECT_H