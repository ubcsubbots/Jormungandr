/*
* Created By: Cameron Newton
* Created On: July 15th, 2018
* Description: Subroutine to detect whether we have found line
*/

#ifndef PROJECT_FINDLINE_H
#define PROJECT_FINDLINE_H

#include "State.h"
#include <constants.h>
#include <line_detect/LineDetectMsg.h>

/*** Communicating Class {findLine, adjustToLine, followLine} ***/
class FindLine : public State {
  public:
    FindLine(std::unordered_map<std::string, double>* constants)
      : State(constants) {}
    std::vector<ros::Subscriber>
    getNodeSubscriptions(ros::NodeHandle nh) override;

  private:
    ros::Timer timer_;

    /**
     * Decides based on image data whether we can see the bottom marker
     *
     * @param msg LineDetectMsg data
     */
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent& event);
};

#endif // PROJECT_FINDLINE_H
