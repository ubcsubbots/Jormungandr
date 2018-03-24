/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description:
 */
#ifndef DECISION_LOCATEGATE_H
#define DECISION_LOCATEGATE_H

#include <std_msgs/String.h>
#include "Subroutine.h"

/*
 * Subroutine: LocateGate
 *
 * Function: Rotates to try to get the gate fully in view
 *
 */
class LocateGate: public Subroutine {
public:
    LocateGate(int argc, char **argv, std::string node_name): Subroutine(argc, argv, node_name) {}
    void setupSubscriptions(ros::NodeHandle nh) override;
private:
    void decisionCallback(const std_msgs::String::ConstPtr& msg);

};

#endif //DECISION_LOCATEGATE_H
