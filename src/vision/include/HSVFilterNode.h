/*
 * Created By: Reid Oliveira
 * Created On: November 11th, 2017
 * Description: Performs an HSV filter on incoming data and republishes it
 */

#ifndef VISION_HSV_H
#define VISION_HSV_H

#include "HSVFilter.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class HSVFilterNode {
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

#endif // VISION_HSV_H
