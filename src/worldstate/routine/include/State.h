//
// Created by joel on 06/05/18.
//

#ifndef PROJECT_ROUTINE_H
#define PROJECT_ROUTINE_H

#include <worldstate/state_msg.h>
#include <ros/ros.h>
/*
 * Documentation to Provide
 *
 * Communication Class: {Starting from this state, it is possible to return through these states}
 * Overaching Goal: {A portion of the RoboSub obstacle course i.e. Gate, Dice, Roulette, Slot, etc.}
 *
 */
class State {
public:

    State(int argc, char** argv, std::string node_name);

    void sleep(void);
    void start(void);

protected:

    ros::Publisher state_publisher_;
    void publishNextState (const worldstate::state_msg &msg);
    virtual void setupNodeSubscriptions (ros::NodeHandle nh) = 0;

};

#endif //PROJECT_ROUTINE_H