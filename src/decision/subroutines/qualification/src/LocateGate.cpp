/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description:
 */

#include "LocateGate.h"

geometry_msgs::Vector3 makeVector(double x, double y, double z) {
    geometry_msgs::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
}

void LocateGate::startup() {

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    nh.subscribe("gate", 10, &LocateGate::decisionCallback, this);

    std::string topic = private_nh.resolveName("gate_location");
    uint32_t queue_size = 1;
    publisher_ = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}

void LocateGate::shutdown() {
    ros::shutdown();
}


void LocateGate::decisionCallback(const std_msgs::String::ConstPtr& msg) {

    // do some processing


    // send the message
    geometry_msgs::Twist command;
    command.angular = makeVector(0.0,0.0,0.0);
    command.linear = makeVector(0.0,0.0,0.0);
    publishCommand(command);

}