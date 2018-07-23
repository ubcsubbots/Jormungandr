/*
 * Created By: Cameron Newton
 * Created On: July 17, 2018
 * Set up environment for LineDetectorNode
 */

#include "LineDetectorNode.h"

LineDetectorNode::LineDetectorNode(int argc, char** argv) {
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it(nh);

    subscribeTopic_ = "camera_output";
    publishTopic_   = "line_detect_output";

    subscriber_ = it.subscribe(
    subscribeTopic_, 2, &LineDetectorNode::subscriberCallBack, this);

    lineDetector_ = LineDetector();

    publisher1_ = nh_.advertise<line_detect::LineDetectMsg>(publishTopic_, 10);

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

    lineImg = cv_ptr->image;

    LinesToFollow linesToFollow = lineDetector_.initialize(lineImg);

    publishLineDetectMsg(linesToFollow);
}

void LineDetectorNode::publishLineDetectMsg(const LinesToFollow linesToFollow) {
    line_detect::LineDetectMsg msg;

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

    publisher1_.publish(msg);
}
