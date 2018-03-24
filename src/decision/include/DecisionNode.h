/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description:
 */
#ifndef DECISION_DECISION_H_H
#define DECISION_DECISION_H_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unordered_map>
#include "Subroutine.h"

// subroutines
// #include "SomeSubroutine.h"


typedef int state_t;

class DecisionNode {
public:
    DecisionNode(int argc, char** argv, std::string node_name);
private:
    std::unordered_map<state_t, Subroutine*> subroutines_;
    Subroutine* running_;
    ros::Subscriber subscriber_;

    void subscriberCallback(state_t state);
    void setupSubroutineMap(int argc, char **argv);
};
#endif //DECISION_DECISION_H_H
