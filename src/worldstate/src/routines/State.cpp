/*
 * Created By:  Joel Ahn
 * Created On:  May 6th, 2018
 * Description: Superclass detailing the individual nodes
 *              of the world state finite state machine
 */


#include "routines/State.h"

State::State(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
}

void State::start() {
    ros::NodeHandle nh;
    setupNodeSubscriptions(nh);

    ros::NodeHandle private_nh("~");

    std::string state_transition_msg =
    private_nh.resolveName("worldstate/output");
    uint32_t queue_size_  = 1;

    state_publisher_ = private_nh.advertise<worldstate::stateMsg>(
    "worldstate/output", queue_size_);
}

void State::sleep() {
    ros::shutdown();
}

void State::publishNextState(const worldstate::stateMsg& msg) {
    state_publisher_.publish(msg);
}
