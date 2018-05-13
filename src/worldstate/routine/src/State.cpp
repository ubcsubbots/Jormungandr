//
// Created by joel on 12/05/18.
//

#include "State.h"

State::State(int argc, char **argv, std::string node_name) {
    ros::init(argc,argv,node_name);
}

void State::start() {
    ros::NodeHandle nh;
    setupNodeSubscriptions(nh);

    ros::NodeHandle private_nh("~");

    std::string state_transition_msg = private_nh.resolveName("worldstate/output");
    int queue_size_                  = 1;
    state_publisher_                 =
    private_nh.advertise<worldstate::state_msg>(state_transition_msg, queue_size_);

    //ros::spin();
}

void State::sleep() {
    ros::shutdown();
}

void State::publishNextState(const worldstate::state_msg &msg) {
    state_publisher_.publish(msg);
}



