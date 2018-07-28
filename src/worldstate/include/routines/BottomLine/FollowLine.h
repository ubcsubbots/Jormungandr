//
// Created by drdc-s2632 on 17/07/18.
//

#ifndef PROJECT_FOLLOWLINE_H
#define PROJECT_FOLLOWLINE_H

#include "State.h"
#include <constants.h>
#include <line_detect/LineDetectMsg.h>

/*** Communicating Class {findLine, adjustToLine, followLine} ***/
class FollowLine : public State {
  public:
    FollowLine(std::unordered_map<std::string, double>* constants)
      : State(constants) {}
    std::vector<ros::Subscriber>
    getNodeSubscriptions(ros::NodeHandle nh) override;

  private:
    ros::Timer timer_;

    /**
    * Decides based on image data whether we can follow the marker
    *
    * @param msg LineDetectMsg data
    */
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent& event);
};

#endif // PROJECT_FOLLOWLINE_H
