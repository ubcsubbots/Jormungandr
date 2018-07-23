/*
 * Created By:  Joel Ahn
 * Created On:  May 6th, 2018
 * Description: Superclass detailing the individual nodes
 *              of the world state finite state machine
 */

#ifndef PROJECT_ROUTINE_H
#define PROJECT_ROUTINE_H

#include <ros/ros.h>
#include <unordered_map>
#include <worldstate/StateMsg.h>

class State {
  private:
    // hold onto these, automatically unsubscribe/unadvertise when out of scope
    ros::Publisher state_publisher_;
    std::vector<ros::Subscriber> subscriptions_;

  public:
    State();

    /**
     * Initializes and starts the state's callback functions
     * so that it is now active
     */
    void start(const std::unordered_map<std::string, double>& constants);

    /**
     * Deactivates a node by shutting down its subscriber
     * so that its callback functions are no longer active
     */
    void sleep();

  protected:
    std::unordered_map<std::string, double> constants_;

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
    virtual std::vector<ros::Subscriber>
    getNodeSubscriptions(ros::NodeHandle nh) = 0;
};

#endif // PROJECT_ROUTINE_H