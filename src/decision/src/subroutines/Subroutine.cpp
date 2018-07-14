/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description: Abstract class for subroutines
 */

#include "Subroutine.h"

Subroutine::Subroutine() {
}

void Subroutine::startup() {
    nh_ = ros::NodeHandle();
    private_nh_ = ros::NodeHandle("~");

    setupSubscriptions(nh_);

    std::string topic   = private_nh_.resolveName("sub_control");
    uint32_t queue_size = 10;
    publisher_ = private_nh_.advertise<geometry_msgs::Twist>(topic, queue_size);
}

void Subroutine::shutdown() {
    nh_.shutdown();
    private_nh_.shutdown();
}

void Subroutine::publishCommand(const geometry_msgs::Twist& msg) {
    publisher_.publish(msg);
}

geometry_msgs::Vector3 Subroutine::makeVector(double x, double y, double z) {
    geometry_msgs::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;

    return v;
}