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
#include "../test/test_utilities/line_detect_test_utils.h"

class LineDetectorNode {
  public:
    LineDetectorNode(int argc, char** argv);


  private:
    image_transport::Subscriber subscriber_;

    image_transport::Publisher test_image_publisher2_;

    ros::Publisher line_msg_publisher;
    image_transport::Publisher line_detected_publish_;
    std::string subscribeTopic_;
    std::string publishTopic_;
    std::string publishTopicTestImage;
    cv::Mat LinesToFollowImage_;
    LineDetector lineDetector_;
    bool displayDetectedLine_;

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

    /**
     * publish lines seen by node and image of the lines
     */
    void publishLineImage( cv::Mat  linesToFollowImage, LineDetector lineDetector, LinesToFollow linesToFollow);

};

#endif // LINE_DETECT_LINEDETECT_H