/*
 * Created By:  Joel Ahn
 * Created On:  May 6th, 2018
 * Description: Superclass detailing the individual nodes
 *              of the world state finite state machine
 */

#include "routines/State.h"

State::State() {
}

void State::start() {
    nh_ = ros::NodeHandle();
    private_nh_ = ros::NodeHandle("~");

    setupNodeSubscriptions(nh_);

    uint32_t queue_size_ = 10;
    // Assign the worldstate publisher
    state_publisher_ = private_nh_.advertise<worldstate::StateMsg>(
    "worldstate/output", queue_size_);
}

void State::sleep() {
    nh_.shutdown();
    private_nh_.shutdown();
}
void State::publishNextState(const worldstate::StateMsg& msg) {
    state_publisher_.publish(msg);
}
