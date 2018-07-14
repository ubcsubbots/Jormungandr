/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description: Subroutine that tries to position the sub in front and
 * orthogonal with the gate, ready to go through.
 */

#ifndef DECISION_LINEUPWITHGATE_H
#define DECISION_LINEUPWITHGATE_H

#include "Subroutine.h"
#include <gate_detect/gateDetectMsg.h>

/*
 * Subroutine: LineUpWithGate
 *
 * Function: attempts to get into an orthogonal position with the gate
 *
 */
class LineUpWithGate : public Subroutine {
  public:
    LineUpWithGate() : Subroutine() {}
    void setupSubscriptions(ros::NodeHandle nh) override;

  private:
    bool allignTop_ = false, distanceToGateAcceptable_ = false;
    void decisionCallback(const gate_detect::gateDetectMsg::ConstPtr& msg);
    void balance(const geometry_msgs::Twist::ConstPtr& msg);
};

#endif // DECISION_LINEUPWITHGATE_H
