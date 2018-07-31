/*
 * Created By: Joel Ahn
 * Created On: March 5h, 2018
 * Description: Manages the various world state nodes,
 *              delegating control to its individual states
 *              and disabling them when the state has changed
 */

#include "routines/BottomLine/AdjustToLine.h"
#include "routines/BottomLine/FindLine.h"
#include "routines/BottomLine/FollowLine.h"
#include "routines/Gate/AlignWithGate.h"
#include "routines/Gate/LocatingGate.h"
#include "routines/Gate/PassGate.h"
#include <WorldStateNode.h>

WorldStateNode::WorldStateNode(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    // Setup NodeHandles
    ros::NodeHandle nh;

    getConstants(nh);
    WorldStateNode::initializeFiniteStateMachine();

    // Change the subscribe topics as needed
    std::string state_transition_msg = "/world_state_node/output";
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
void WorldStateNode::initializeFiniteStateMachine() {
    // Gate
    state_machine_[worldstate::StateMsg::adjustingDepth] =
    new AdjustToLine(&constants_);

    state_machine_[worldstate::StateMsg::locatingGate] =
    new LocatingGate(&constants_);
    state_machine_[worldstate::StateMsg::approachingGate] =
    new ApproachGate(&constants_);
    state_machine_[worldstate::StateMsg::aligningWithGate] =
    new AlignWithGate(&constants_);
    state_machine_[worldstate::StateMsg::passingGate] =
    new PassGate(&constants_);

    // Line Following
    state_machine_[worldstate::StateMsg::followingLine] =
    new FindLine(&constants_);
    state_machine_[worldstate::StateMsg::adjustingToLine] =
    new AdjustToLine(&constants_);
    state_machine_[worldstate::StateMsg::followingLine] =
    new FollowLine(&constants_);

    // Activate the state_machine with the initial state
    current_state_ = state_machine_[initial_state_];
    current_state_->start();
}

void WorldStateNode::getConstants(ros::NodeHandle nh) {
    XmlRpc::XmlRpcValue v;

    nh.getParam("/global_constants", v);

    for (auto value = v.begin(); value != v.end(); value++) {
        constants_[std::string((*value).first)] = (*value).second;
    }
}
