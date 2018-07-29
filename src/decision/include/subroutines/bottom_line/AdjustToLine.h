/*
* Created By: Cameron Newton
* Created On: July 15th, 2018
* Description: Subroutine to adjust to bottom marker
*/

#ifndef PROJECT_ADJUSTTOLINE_H
#define PROJECT_ADJUSTTOLINE_H

#include "Subroutine.h"
#include <line_detect/LineDetectMsg.h>

class AdjustToLine : public Subroutine {
  public:
    AdjustToLine(std::unordered_map<std::string, double>* constants)
      : Subroutine(constants) {}
    std::string getName() override { return "AdjustToLine"; }

    std::vector<ros::Subscriber> getSubscriptions(ros::NodeHandle nh) override;

  private:
    /**
     * Decides based on image data whether the robot still needs to
     * align with the bottom marker.
     *
     * @param msg gateDetectMsg data
     */
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_ADJUSTTOLINE_H
