/*
 * Created By:  Joel Ahn
 * Created On:  May 6th, 2018
 * Description: Superclass detailing the individual nodes
 *              of the world state finite state machine
 */

#include "routines/State.h"

State::State() {}

void State::start(const std::unordered_map<std::string, double>& constants) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    constants_     = constants;
    subscriptions_ = getNodeSubscriptions(nh);

    uint32_t queue_size_ = 10;
    // Assign the worldstate publisher
    state_publisher_ =
    private_nh.advertise<worldstate::StateMsg>("output", queue_size_);
}

void State::sleep() {
    state_publisher_.shutdown();

    for (ros::Subscriber subscription : subscriptions_) {
        subscription.shutdown();
    }
    subscriptions_.clear();
}
void State::publishNextState(const worldstate::StateMsg& msg) {
    state_publisher_.publish(msg);
}
