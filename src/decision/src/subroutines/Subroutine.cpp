/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description: Abstract class for subroutines
 */

#include "Subroutine.h"

Subroutine::Subroutine(std::unordered_map<std::string, double>* constants) {
    constants_ = constants;
}

void Subroutine::startup() {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    subscriptions_ = getSubscriptions(nh);

    std::string topic   = private_nh.resolveName("output");
    uint32_t queue_size = 10;
    publisher_ = private_nh.advertise<nav_msgs::Odometry>(topic, queue_size);
}

void Subroutine::shutdown() {
    publisher_.shutdown();

    for (ros::Subscriber subscription : subscriptions_) {
        subscription.shutdown();
    }
    subscriptions_.clear();
}

void Subroutine::publishCommand(const nav_msgs::Odometry& msg) {
    publisher_.publish(msg);
}

geometry_msgs::Vector3 Subroutine::makeVector(double x, double y, double z) {
    geometry_msgs::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;

    return v;
}