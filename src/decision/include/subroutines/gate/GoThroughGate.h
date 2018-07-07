/*
 * Created By: Reid Oliveira
 * Created On: May 19, 2018
 * Description: Subroutine that simply proceeds forward through the gate
 */

#ifndef DECISION_GOTHROUGHGATE_H
#define DECISION_GOTHROUGHGATE_H

#include "Subroutine.h"
#include <gate_detect/GateDetectMsg.h>

/*
 * Subroutine: GoThroughGate
 *
 * Function: proceeds forward through the gate
 *
 */
class GoThroughGate : public Subroutine {
  public:
    GoThroughGate(int argc, char** argv, std::string node_name)
      : Subroutine(argc, argv, node_name) {}
    void setupSubscriptions(ros::NodeHandle nh) override;
    void sleep() override;

  private:
    void decisionCallback(const gate_detect::GateDetectMsg::ConstPtr& msg);
};

#endif // DECISION_GOTHROUGHGATE_H
