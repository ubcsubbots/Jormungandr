/*
 * Created By: Cameron Newton
 * Created On: May 19, 2018
 * Description: Subroutine that goes towards the gate until we're close enough
 * to align
 */

#ifndef DECISION_APPROACHGATE_H
#define DECISION_APPROACHGATE_H

#include "Subroutine.h"
#include <gate_detect/GateDetectMsg.h>

/*
 * Subroutine: GoThroughGate
 *
 * Function: proceeds forward through the gate
 *
 */
class ApproachGate : public Subroutine {
  public:
    ApproachGate(std::unordered_map<std::string, double>* constants)
      : Subroutine(constants) {}
    std::string getName() override { return "ApproachGate"; }

    std::vector<ros::Subscriber> getSubscriptions(ros::NodeHandle nh) override;

  private:
    void decisionCallback(const gate_detect::GateDetectMsg::ConstPtr& msg);
};

#endif // DECISION_APPROACHGATE_H
