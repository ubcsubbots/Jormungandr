/*
 * Created By: Joel Ahn
 * Created On: March 5h, 2018
 * Description: Checks the incoming data from detection nodes
 *              and tracks the individual goals of the robot
 */

#ifndef JORMUNGANDR_WORLDSTATENODE_H
#define JORMUNGANDR_WORLDSTATENODE_H

#include <ros/ros.h>
#include <worldstate/state_msg.h>
#include <std_msgs/builtin_int8.h>
#include <gate_detect/gateDetectMsg.h>

class WorldStateNode {

public:
    WorldStateNode(int argc, char** argv, std::string node_name);

private:
    /**
     * TODO: Change the parameter to receive gate detection topics
     * Callback function for when data is received from gate detection node
     *
     * @param gate detection node discretized messages
     */
    void gateDetectCallBack(const gate_detect::gateDetectMsgConstPtr & gateDetMsg);

    enum internalWorldStates {
        locatingGate,
        aligningWithGate,
        passingGate,
//        locatingPole,
//        approachingPole,
//        pivotingPole
    };

    /* Temporarily directly receive hsv-filtered msgs */
    //image_transport::Subscriber gate_node_subscriber_;
    ros::Subscriber gate_detect_listener_;
    ros::Publisher world_state_publisher_;

    /*
     * Configurable parameters adjusted according to the physical dimensions
     * of the robot and the competition surface
     */
    const int kClearanceGateHeight = 16;

    /* Robot should always start off by searching for the starting gate*/
    internalWorldStates current_state_ = locatingGate;

};

#endif //JORMUNGANDR_WORLDSTATENODE_H
