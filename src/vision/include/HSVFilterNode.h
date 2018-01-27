/*
 * Created By: Reid Oliveira
 * Created On: November 11th, 2017
 * Description: Performs an HSV filter on incoming data and republishes it
 */

#ifndef PROJECT_HSV_H
#define PROJECT_HSV_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "HSVFilter.h"

class HSVFilterNode {
    static const std::string kSubscribeTopic;
    static const std::string kPublishTopic;
    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher_;
    HSVFilter filter_;

public:
    HSVFilterNode(int argc, char** argv, std::string node_name);

private:
    /**
     * Callback function for when a new image is received
     *
     * @param image the image received in the callback
     */
    void subscriberCallBack(const sensor_msgs::ImageConstPtr& image);
    /**
     * Publishes the filtered image
     *
     * @param image the image to publish
     */
    void publishFilteredImage(const cv::Mat& filtered_image);
};

#endif //PROJECT_HSV_H