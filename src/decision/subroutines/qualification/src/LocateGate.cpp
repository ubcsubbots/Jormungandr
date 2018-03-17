/*
 * Created By: reidoliveira
 * Created On: March 17, 2018
 * Description:
 */

#include "LocateGate.h"

void LocateGate::startup() {

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    setupSubscriptions(nh);

    std::string topic = private_nh.resolveName("");
    uint32_t queue_size = 1;
    publisher_ = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}

void LocateGate::shutdown() {
    ros::shutdown();
}

void LocateGate::publishCommand(const geometry_msgs::Twist::ConstPtr &msg) {

}

void LocateGate::setupSubscriptions(ros::NodeHandle nh) {
    subscriptions_.push_back(nh.subscribe("first", 10, &LocateGate::firstSubscriberCallback, this));

}

void LocateGate::firstSubscriberCallback(const std_msgs::String::ConstPtr& msg) {

}