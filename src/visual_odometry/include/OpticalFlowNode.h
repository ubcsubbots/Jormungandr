/*
 * Created By: Kevin Huang
 * Created On: February 2nd, 2020
 * Description: Performs visual odometry
 */

#ifndef VISUAL_ODOMETRY_OPTICALFLOWNODE_H
#define VISUAL_ODOMETRY_OPTICALFLOWNODE_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#define MAX_CORNERS 500
#define RANSAC_ITERATIONS 3000

using namespace std;
using namespace cv;

class OpticalFlowNode {
    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher_;
    Mat prev_frame; //Stores last image
    vector<Point2f> prev_points[5]; //Stores features from last five frames
    bool need_init;

public:
    OpticalFlowNode(int argc, char** argv, std::string node_name);

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
    void publishTracking(const cv::Mat& filtered_image);
};

#endif //VISUAL_ODOMETRY_OPTICALFLOWNODE_H