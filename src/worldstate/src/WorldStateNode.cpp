/*
 * Created By: Joel Ahn
 * Created On: March 5h, 2018
 * Description: Manages the various world state nodes,
 *              delegating control to its individual states
 *              and disabling them when the state has changed
 */

#include "routines/Gate/AlignWithGate.h"
#include "routines/Gate/LocatingGate.h"
#include "routines/Gate/PassGate.h"
#include <WorldStateNode.h>

WorldStateNode::WorldStateNode(int argc, char** argv, std::string node_name) {
    WorldStateNode::initializeFiniteStateMachine(argc, argv);

    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Change the subscribe topics as needed
    std::string state_transition_msg = "worldstate/output";
    uint32_t refresh_rate            = 10;
    world_state_listener_            = nh.subscribe(state_transition_msg,
                                         refresh_rate,
                                         &WorldStateNode::stateChangeCallBack,
                                         this);
}

/**
 * Callback function when a message is received from
 * a node of the world state FSM. Enables and disables
 * nodes.
 *
 * @param msg message containing the current state
 */
void WorldStateNode::stateChangeCallBack(
const worldstate::StateMsg::ConstPtr& msg) {
    state_t new_state_index_ = msg->state;

    State* nextState = state_machine_.at(new_state_index_);

    if (nextState != current_state_) {
        current_state_->sleep();
        current_state_ = nextState;
        current_state_->start();
    }
}

/**
 * Sets up the map "routines" such that each enumerated state is mapped to
 * its appropriate subroutine.
 * @param argc standard argc passed in from main, used for the ros::init of each
 * subroutine
 * @param argv standard argv passed in from main, used for the ros::init of each
 * subroutine
 */
void WorldStateNode::initializeFiniteStateMachine(int argc, char** argv) {
    state_machine_[worldstate::StateMsg::locatingGate] =
    new LocatingGate(argc, argv, "locating_gate_ws");

    state_machine_[worldstate::StateMsg::aligningWithGate] =
    new AlignWithGate(argc, argv, "aligning_with_gate_ws");

    state_machine_[worldstate::StateMsg::passingGate] =
    new PassGate(argc, argv, "passing_thru_gate_ws");

    // Activate the state_machine with the initial state
    current_state_ = state_machine_[initial_state_];
    current_state_->start();
}
