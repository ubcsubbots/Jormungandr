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
 * Make sure to provide for each State subclass the possible
 * StateMsg types that it can publish i.e. to which other States
 * that the World Finite State Machine can transition from
 * the current one to. Describe before the class declaration as such:
 *
/*** Communicating States {States that it can possibly transition to} ***/
class State {
  public:
    State(int argc, char** argv, std::string node_name);

    /**
     * Initializes and starts the state's callback functions
     * so that it is now active
     */
    void start();

    /**
     * Deactivates a node by shutting down its subscriber
     * so that its callback functions are no longer active
     */
    virtual void sleep() = 0;

  protected:
    ros::Publisher state_publisher_;

    /**
     * Publishes the next state in the finite state machine
     * to transition to
     *
     * @param msg contains the next state to transition to
     */
    void publishNextState(const worldstate::StateMsg& msg);

    /**
     * Describes the topics that the State is subscribed to.
     * Nodehandle subscriptions should be returned and assigned
     * to ros Subscribers in this function
     *
     * @param nh the private nodehandle of the State
     */
    virtual void setupNodeSubscriptions(ros::NodeHandle nh) = 0;
};

#endif // PROJECT_ROUTINE_H