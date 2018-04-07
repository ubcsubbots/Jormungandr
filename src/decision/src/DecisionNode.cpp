/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description:
 */
#include "DecisionNode.h"

DecisionNode::DecisionNode(int argc, char **argv, std::string node_name) {
    setupSubroutineMap(argc, argv);

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string state_topic = "worldstate";
    int refresh_rate = 10;
    subscriber_ = nh.subscribe(state_topic, refresh_rate, &DecisionNode::subscriberCallback, this);
}

void DecisionNode::subscriberCallback(const worldstate::state_msg::ConstPtr& msg) {

    state_t state = msg->state;

    if(subroutines_.find(state) == subroutines_.end()) {
        // We forgot to add a subroutine to the map. This is bad.
        // TODO do something
        return;
    }

    Subroutine *newState = subroutines_[state];
    if(newState == running_) {
        return;
    }

    running_->shutdown();
    running_ = newState;
    running_->startup();

}

void DecisionNode::setupSubroutineMap(int argc, char **argv) {
    subroutines_[worldstate::state_msg::locatingGate] = new LocateGate(argc, argv, "locate_gate");
    subroutines_[worldstate::state_msg::aligningWithGate] = new LineUpWithGate(argc, argv, "align_with_gate");
}
