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

/*
 * Specifically we need to discuss if we want to handle the world state with a
 * similar design pattern to the decision maker (submodules), how we want to handle
 * having multiple message inputs into it/manage subscriptions,
 * and then somewhat of a similar discussion with things like HSV filters,
 * i.e. if we want to always be looking for the gate or eventually shut that down
 */
class WorldStateNode {

public:
    WorldStateNode(int argc, char** argv, std::string node_name);

private:
    /**
     * Callback function for when data is received from gate detection node
     *
     * @param gate detection node discretized messages
     */
    void gateDetectCallBack(const gate_detect::gateDetectMsgConstPtr & msg);

    /**
     * Callback function for when data is received from pole detection node
     *
     * @param gate detection node discretized messages
     */
    void poleDetectCallBack(void);

    enum internalWorldStates {
        locatingGate,
        aligningWithGate,
        passingGate,
        locatingPole,
        approachingPole,
        pivotingPole
    };

    /* Temporarily directly receive hsv-filtered msgs */
    //image_transport::Subscriber gate_node_subscriber_;
    ros::Subscriber gate_detect_listener_;
    ros::Publisher world_state_publisher_;

    /* Robot should always start off by searching for the starting gate*/
    internalWorldStates current_state_ = locatingGate;

};

#endif //JORMUNGANDR_WORLDSTATENODE_H
