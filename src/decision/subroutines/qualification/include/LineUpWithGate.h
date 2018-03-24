/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description:
 */
#ifndef DECISION_LINEUPWITHGATE_H
#define DECISION_LINEUPWITHGATE_H

#include <std_msgs/String.h>
#include "Subroutine.h"

/*
 * Subroutine: LineUpWithGate
 *
 * Function: attempts to get into an orthogonal position with the gate
 *
 */
class LineUpWithGate: public Subroutine {
public:
    LineUpWithGate(int argc, char **argv, std::string node_name): Subroutine(argc, argv, node_name) {}
    void setupSubscriptions(ros::NodeHandle nh) override;
private:
    void decisionCallback(const std_msgs::String::ConstPtr& msg);

};

#endif //DECISION_LINEUPWITHGATE_H
