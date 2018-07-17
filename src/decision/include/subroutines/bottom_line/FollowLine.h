//
// Created by drdc-s2632 on 17/07/18.
//

#ifndef PROJECT_FOLLOWLINE_H
#define PROJECT_FOLLOWLINE_H

#include "Subroutine.h"
#include <line_detect/LineDetectMsg.h>

/*** Communicating Class {findLine, adjustToLine, followLine} ***/
class FollowLine : public Subroutine {
public:
    FollowLine() : Subroutine() {}
    std::string getName() override { return "FollowLine"; }

    std::vector<ros::Subscriber> getSubscriptions(ros::NodeHandle nh) override;

private:
    /**
     * Decides based on image data whether the robot still needs to
     * align with the gate.
     *
     * @param msg gateDetectMsg data
     */
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);
};

#endif //PROJECT_FOLLOWLINE_H
