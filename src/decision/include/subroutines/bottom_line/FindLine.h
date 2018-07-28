/*
* Created By: Cameron Newton
* Created On: July 15th, 2018
* Description: Subroutine to fine bottom marker
*/

#ifndef PROJECT_FINDLINE_H
#define PROJECT_FINDLINE_H

#include "Subroutine.h"
#include <line_detect/LineDetectMsg.h>

class FindLine : public Subroutine {
  public:
    FindLine(std::unordered_map<std::string, double>* constants)
      : Subroutine(constants) {}
    std::string getName() override { return "FindLine"; }

    std::vector<ros::Subscriber> getSubscriptions(ros::NodeHandle nh) override;

  private:
    ros::Timer timer_;

    // Directional variable that will be negated when robot cannot find line,
    // causes robot to change direction
    double lateralVelocityDirection_;

    /**
     * Callback from line_detect msg that will determine whether we have found
     * the line or not, robot will
     * periodically reverse direction it that it's looking for the line
     *
     * @param msg gateDetectMsg data
     */
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);

    /**
     * Reverses direction that robot travels to find line
     *
     * @param event event that triggers timer
     */
    void timerCallback(const ros::TimerEvent& event);
};

#endif // PROJECT_FINDLINE_H
