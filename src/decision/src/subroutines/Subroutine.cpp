/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description: Abstract class for subroutines
 */

#include "Subroutine.h"

Subroutine::Subroutine(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
}

void Subroutine::startup() {
    ros::NodeHandle nh;
    setupSubscriptions(nh);

    ros::NodeHandle private_nh("~");
    std::string topic   = private_nh.resolveName("sub_control");
    uint32_t queue_size = 1;
    publisher_ = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}

void Subroutine::shutdown() {
    ros::shutdown();
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