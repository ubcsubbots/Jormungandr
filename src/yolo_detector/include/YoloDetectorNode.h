/*
 * Created By: Kevin Huang
 * Created On: June 26, 2020
 * Description: Performs YOLO object detection
 */

#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class YoloDetectorNode {
    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher_;

  public:
    YoloDetectorNode(int argc, char** argv, std::string node_name);

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
    void publishDetected(const cv::Mat& filtered_image);

    void preprocess(const cv::Mat& frame, cv::dnn::Net& net, cv::Size inpSize, float scale,
                    const cv::Scalar& mean, bool swapRB);

    void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs, cv::dnn::Net& net);

    void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

    cv::dnn::Net yolo_net;

    float confThreshold = 0.2;
    float nmsThreshold = 0.2;
    float scale = 0.00392;
    int inpWidth = 416;
    int inpHeight = 416;
    bool swapRB = true;
    float mean = 0.5;
    std::vector<std::string> classes;
};

#endif