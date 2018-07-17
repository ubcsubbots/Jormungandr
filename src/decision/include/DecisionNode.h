/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description: Node responsible for making navigation decisions. Invokes a
 * subroutine for each logical state of
 * operation.
 */

#ifndef DECISION_DECISION_H_H
#define DECISION_DECISION_H_H

#include "Subroutine.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <unordered_map>

// state msg
#include <worldstate/StateMsg.h>
// subroutines
#include "ApproachGate.h"
#include "GoThroughGate.h"
#include "LineUpWithGate.h"
#include "LocateGate.h"

typedef int8_t state_t;

class DecisionNode {
  public:
    DecisionNode(int argc, char** argv, std::string node_name);
    const char* logger_name = ROSCONSOLE_DEFAULT_NAME ".<name>";

  private:
    std::unordered_map<state_t, Subroutine*>
    subroutines_;         // holds all of the known subroutines
    Subroutine* running_; // the currently running subroutine
    ros::Subscriber worldstate_subscriber_; // subscribes to the world state

    // says which subroutine is running, for testing & debugging
    ros::Publisher info_publisher_;

    /**
     * Callback function when a message is received from the world state node.
     * @param state_msg message containing the current state
     */
    void worldStateCallback(const worldstate::StateMsg::ConstPtr& state_msg);

    /**
     * Sets up the map "subroutines_" such that each enumerated state is mapped
     * to its appropriate subroutine.
     */
    void setupSubroutineMap();
};
#endif // DECISION_DECISION_H_H
