/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description: Subroutine that attempts to position the gate so that it can be
 * seen by the camera.
 */
#ifndef DECISION_LOCATEGATE_H
#define DECISION_LOCATEGATE_H

#include "Subroutine.h"
#include <gate_detect/gateDetectMsg.h>
#include <std_msgs/String.h>

/*
 * Subroutine: LocateGate
 *
 * Function: Rotates to try to get the gate fully in view
 *
 */
class LocateGate : public Subroutine {
  public:
    LocateGate(int argc, char** argv, std::string node_name)
      : Subroutine(argc, argv, node_name) {}
    void setupSubscriptions(ros::NodeHandle nh) override;

  private:
    void decisionCallback(const gate_detect::gateDetectMsg::ConstPtr& msg);
};

#endif // DECISION_LOCATEGATE_H
