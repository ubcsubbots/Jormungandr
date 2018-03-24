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

    std::string state_topic = "states";
    int refresh_rate = 10;
    subscriber_ = nh.subscribe(state_topic, refresh_rate, &DecisionNode::subscriberCallback, this);
}

void DecisionNode::subscriberCallback(const std_msgs::Int32::ConstPtr& msg) {

    state_t state = msg->data;

    Subroutine *newState = subroutines_[state];
    if(newState == running_) {
        return;
    }

    running_->shutdown();
    running_ = newState;
    running_->startup();

}

void DecisionNode::setupSubroutineMap(int argc, char **argv) {
    // subroutines_[enumId] = new SomeSubroutine(argc, argv, "someSubroutine");

}
