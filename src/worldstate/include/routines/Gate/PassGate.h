/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to pass gate.
 */

#ifndef PROJECT_PASSGATE_H
#define PROJECT_PASSGATE_H

#include "State.h"
#include "constants.h"
#include <gate_detect/GateDetectMsg.h>
#include <line_detect/LineDetectMsg.h>

/*** Communicating Class {alignWithGate, locatingGate, passGate} ***/
class PassGate : public State {
public:
    PassGate() : State() {}
    std::vector<ros::Subscriber>
    getNodeSubscriptions(ros::NodeHandle nh) override;

private:
    ros::Timer timer_;

    /**
     * Decides based on image data whether the robot still needs to
     * align with the gate.
     *
     * @param msg gateDetectMsg data
     */
    void gateDetectCallBack(const gate_detect::GateDetectMsg::ConstPtr& msg);

    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent& event);

};

#endif // PROJECT_PASSGATE_H
