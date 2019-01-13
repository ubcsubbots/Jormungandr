//
// Created by viral on 03/01/19.
//

#ifndef PROJECT_ACCUMULATORNODE_H
#define PROJECT_ACCUMULATORNODE_H

#include "Accumulator.h"
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class AccumulatorNode {
    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher_;
    Accumulator accumulator_;

public:
    AccumulatorNode(int argc, char** argv, std::string node_name);


private:
    /**
    * Callback function for when a new image is received
    *
    * @param image the image received in the callback
    */
    void subscriberCallBack(const sensor_msgs::ImageConstPtr& image);
    /**
     * Publishes the mask
     *
     * @param mask: the mask to publish
     */
    void publishMask(const cv::Mat& mask);


};


#endif //PROJECT_ACCUMULATORNODE_H
