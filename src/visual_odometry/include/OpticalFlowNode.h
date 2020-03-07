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
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
//#include <opengv/sac/Ransac.hpp>
//#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#define MAX_CORNERS 500
#define RANSAC_ITERATIONS 3000
#define RETRACK_THRESH 75
#define KEYFRAME_THRESH 10
#define KLT_QUALITY 0.01
#define KLT_MIN_DIST 20
#define STORED_FRAMES 5
#define STORED_KEYFRAMES 5

using namespace std;
using namespace cv;
//using namespace Eigen;
//using namespace opengv;

class OpticalFlowNode {
    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher_;
    Mat prev_frame[STORED_FRAMES]; //Stores last image
    Mat prev_keyframe[STORED_KEYFRAMES];
    vector<Point2f> prev_keypoints[STORED_KEYFRAMES];
    vector<Point2f> prev_points[STORED_FRAMES]; //Stores features from last five frames
    vector<Point2f> prev_lost[STORED_FRAMES]; //Stores lost features
    bool need_init;
    Mat camera_matrix = (Mat_<double>(3,3) << 243.63,0,404.45,0,327.26,203.98,0,0,1);

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