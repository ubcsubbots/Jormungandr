/*
 * Created By:  Joel Ahn
 * Created On:  May 6th, 2018
 * Description: Superclass detailing the individual nodes
 *              of the world state finite state machine
 */

#ifndef PROJECT_ROUTINE_H
#define PROJECT_ROUTINE_H

#include <ros/ros.h>
#include <worldstate/StateMsg.h>
/*
 * Documentation to Provide
 *
 * Communicating States {States that it can possibly transition to}
 *
 */
class State {
  public:
    State(int argc, char** argv, std::string node_name);

    virtual void sleep() = 0;
    void start();

  protected:
    ros::Publisher  state_publisher_;

    /**
     * Publishes at each clock the next state in the finite state
     * machine to transition to
     *
     * @param msg contains the next state to transition to
     */
    void publishNextState(const worldstate::StateMsg& msg);

    virtual void setupNodeSubscriptions(ros::NodeHandle nh) = 0;
};

#endif // PROJECT_ROUTINE_H