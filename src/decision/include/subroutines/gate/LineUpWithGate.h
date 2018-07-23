/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description: Subroutine that tries to position the sub in front and
 * orthogonal with the gate, ready to go through.
 */

#ifndef DECISION_LINEUPWITHGATE_H
#define DECISION_LINEUPWITHGATE_H

#include "Subroutine.h"
#include <gate_detect/GateDetectMsg.h>

/*
 * Subroutine: LineUpWithGate
 *
 * Function: attempts to get into an orthogonal position with the gate
 *
 */
class LineUpWithGate : public Subroutine {
  public:
    LineUpWithGate(std::unordered_map<std::string, double>* constants)
      : Subroutine(constants) {}
    std::string getName() override { return "LineUpWithGate"; }

    std::vector<ros::Subscriber> getSubscriptions(ros::NodeHandle nh) override;

  private:
    void decisionCallback(const gate_detect::GateDetectMsg::ConstPtr& msg);
};

#endif // DECISION_LINEUPWITHGATE_H
