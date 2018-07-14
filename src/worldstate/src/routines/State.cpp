/*
 * Created By:  Joel Ahn
 * Created On:  May 6th, 2018
 * Description: Superclass detailing the individual nodes
 *              of the world state finite state machine
 */

#include "routines/State.h"

State::State() {}

void State::start() {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    setupNodeSubscriptions(nh);

    uint32_t queue_size_ = 10;
    // Assign the worldstate publisher
    state_publisher_ = private_nh.advertise<worldstate::StateMsg>(
    "output", queue_size_);
}

void State::sleep() {
    state_publisher_.shutdown();
    subscriber_.shutdown();
}
void State::publishNextState(const worldstate::StateMsg& msg) {
    state_publisher_.publish(msg);
}
