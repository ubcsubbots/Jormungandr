//
// Created by drdc-s2632 on 17/07/18.
//

#ifndef PROJECT_FINDLINE_H
#define PROJECT_FINDLINE_H

#include "State.h"
#include <constants.h>
#include <line_detect/LineDetectMsg.h>

/*** Communicating Class {findLine, adjustToLine, followLine} ***/
class FindLine : public State {
public:
    FindLine() : State() {}
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
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent& event);
};

#endif //PROJECT_FINDLINE_H
