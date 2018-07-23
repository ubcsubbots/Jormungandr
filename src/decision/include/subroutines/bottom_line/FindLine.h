//
// Created by drdc-s2632 on 17/07/18.
//

#ifndef PROJECT_FINDLINE_H
#define PROJECT_FINDLINE_H

#include "Subroutine.h"
#include <line_detect/LineDetectMsg.h>

/*** Communicating Class {findLine, adjustToLine, followLine} ***/
class FindLine : public Subroutine {
  public:
    FindLine(std::unordered_map<std::string, double>* constants)
      : Subroutine(constants) {}
    std::string getName() override { return "FindLine"; }

    std::vector<ros::Subscriber> getSubscriptions(ros::NodeHandle nh) override;

  private:
    ros::Timer timer_;

    double lateralVelocityDirection_;

    /**
     * Decides based on image data whether the robot still needs to
     * align with the gate.
     *
     * @param msg gateDetectMsg data
     */
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent& event);
};

#endif // PROJECT_FINDLINE_H
