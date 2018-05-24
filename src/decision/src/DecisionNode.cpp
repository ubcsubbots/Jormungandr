/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description: Node responsible for making navigation decisions. Invokes a
 * subroutine for each logical state of
 * operation.
 */
#include "DecisionNode.h"

DecisionNode::DecisionNode(int argc, char** argv, std::string node_name) {
    setupSubroutineMap(argc, argv);

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string state_topic = "worldstate";
    int refresh_rate        = 10;
    worldstate_subscriber_  = nh.subscribe(
    state_topic, refresh_rate, &DecisionNode::worldStateCallback, this);
}

/**
 * Callback function when a message is received from the world state node.
 * @param StateMsg message containing the current state
 */
void DecisionNode::worldStateCallback(
const worldstate::StateMsg::ConstPtr& StateMsg) {
    state_t state = StateMsg->state;

    if (subroutines_.find(state) == subroutines_.end()) {
        // We forgot to add a subroutine to the map. This is bad.

        ROS_ERROR_STREAM(state
                         << " was not found in the map of known subroutines");
        ros::shutdown();
    }

    Subroutine* newState = subroutines_[state];
    if (newState == running_) { return; }

    running_->shutdown();
    running_ = newState;
    running_->startup();
}

/**
 * Sets up the map "subroutines_" such that each enumerated state is mapped to
 * its appropriate subroutine.
 * @param argc standard argc passed in from main, used for the ros::init of each
 * subroutine
 * @param argv standard argv passed in from main, used for the ros::init of each
 * subroutine
 */
void DecisionNode::setupSubroutineMap(int argc, char** argv) {
    subroutines_[worldstate::StateMsg::locatingGate] =
    new LocateGate(argc, argv, "locate_gate");
    subroutines_[worldstate::StateMsg::aligningWithGate] =
    new LineUpWithGate(argc, argv, "align_with_gate");
}
