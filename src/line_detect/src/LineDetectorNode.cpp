/*
 * Created By: Cameron Newton
 * Created On: July 17, 2018
 * LineDetectorNode takes in a filter_image and outputs whether it sees a marker
 * on the bottom of the pool
 * Output message is of type LineDetectMsg, definition is in LineDetectMsg file
 */

#include "LineDetectorNode.h"

LineDetectorNode::LineDetectorNode(int argc, char** argv) {
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it(nh);

    subscribeTopic_ = "camera_output";
    publishTopic_   = "line_detect_output";
    publishTopicTestImage = "test_image_line_detect";

    displayDetectedLine_ = true;

    nh.getParam("/line_detect_node/displayDetectedLine", displayDetectedLine_);

    subscriber_ = it.subscribe(
    subscribeTopic_, 2, &LineDetectorNode::subscriberCallBack, this);

    lineDetector_ = LineDetector();

    line_msg_publisher =
    nh_.advertise<line_detect::LineDetectMsg>(publishTopic_, 10);
    line_detected_publish_ = it.advertise("line_debug_image",100);

    test_image_publisher2_     = it.advertise(publishTopicTestImage, 1);

    ros::spin();
}

void LineDetectorNode::subscriberCallBack(
const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    LinesToFollowImage_ = cv_ptr->image;

    LinesToFollow linesToFollow = lineDetector_.initialize(LinesToFollowImage_);
    if(displayDetectedLine_)
        publishLineImage(LinesToFollowImage_, lineDetector_, linesToFollow );

    publishLineDetectMsg(linesToFollow);
}

void LineDetectorNode::publishLineDetectMsg(const LinesToFollow linesToFollow) {
    line_detect::LineDetectMsg msg;

    // Build LineDetectMsg based on output from LineDetector, struct definition
    // in LineDetect.h
    // Transforms LinesToFollow struct into LineDetectMsg using LineDetect
    // functions
    msg.lateralDistanceFromFrontMarker = lineDetector_.calcProjectedDistance(
    linesToFollow.frontLine.middleOfMarker, linesToFollow.frontLine.width);
    msg.distanceFromEndOfFrontMarker =
    lineDetector_.calcProjectedDistanceToEndOfLine(
    linesToFollow.frontLine.frontOfMarker, linesToFollow.frontLine.width);
    msg.angleToParallelFrontMarker = linesToFollow.frontLine.slope;

    msg.lateralDistanceFromRearMarker = lineDetector_.calcProjectedDistance(
    linesToFollow.rearLine.middleOfMarker, linesToFollow.rearLine.width);
    msg.distanceFromEndOfFrontMarker =
    lineDetector_.calcProjectedDistanceToEndOfLine(
    linesToFollow.rearLine.frontOfMarker, linesToFollow.rearLine.width);
    msg.angleToParallelRearMarker = linesToFollow.rearLine.slope;

    line_msg_publisher.publish(msg);
}

void LineDetectorNode::publishLineImage( cv::Mat  linesToFollowImage, LineDetector lineDetector, LinesToFollow linesToFollow){
    //cv::Mat Lines;

    //cv::cvtColor(linesToFollowImage, linesToFollowImage, CV_GRAY2BGR);

    linesToFollowImage = TestUtils::drawLineToFollow(linesToFollowImage, lineDetector, linesToFollow);

    cv_bridge::CvImage out_msg;
    out_msg.header =
            std_msgs::Header(); // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    out_msg.image    = linesToFollowImage;                          // Your cv::Mat
    test_image_publisher2_.publish(out_msg.toImageMsg());
}