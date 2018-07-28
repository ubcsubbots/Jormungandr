/*
* Created By: Cameron Newton
* Created On: July 15th, 2018
* Description: Subroutine to follow line
*/
#ifndef PROJECT_FOLLOWLINE_H
#define PROJECT_FOLLOWLINE_H

#include "Subroutine.h"
#include <line_detect/LineDetectMsg.h>

class FollowLine : public Subroutine {
  public:
    FollowLine(std::unordered_map<std::string, double>* constants)
      : Subroutine(constants) {}

    std::string getName() override { return "FollowLine"; }

    std::vector<ros::Subscriber> getSubscriptions(ros::NodeHandle nh) override;

  private:
    /**
     * Robot will proceed forward and follow line
     *
     * @param msg LineDetect msg
     */
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_FOLLOWLINE_H
