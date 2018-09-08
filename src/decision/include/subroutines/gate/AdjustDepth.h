/*
 * Created By: Cameron Newton
 * Created On: July 19, 2018
 * Description: Subroutine that adjusts depth of robot
 * to align
 */

#ifndef PROJECT_ADJUSTHEIGHT_H
#define PROJECT_ADJUSTHEIGHT_H

#include "Subroutine.h"
#include <gate_detect/GateDetectMsg.h>

/*
 * Subroutine: AdjustHeight
 *
 * Function: Adjust depth of robot
 *
 */
class AdjustDepth : public Subroutine {
  public:
    AdjustDepth(std::unordered_map<std::string, double>* constants)
      : Subroutine(constants) {}
    std::string getName() override { return "ApproachGate"; }

    std::vector<ros::Subscriber> getSubscriptions(ros::NodeHandle nh) override;

  private:
    void decisionCallback(const gate_detect::GateDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_ADJUSTHEIGHT_H
