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
    ros::NodeHandle private_nh;

    setupNodeSubscriptions(private_nh);

    // All states should communicate with the world_state_node
    std::string state_transition_msg =
    private_nh.resolveName("worldstate/output");
    uint32_t queue_size_ = 10;
    // Assign the worldstate publisher
    state_publisher_ = private_nh.advertise<worldstate::StateMsg>(
    "worldstate/output", queue_size_);
}

void State::publishNextState(const worldstate::StateMsg& msg) {
    state_publisher_.publish(msg);
}
