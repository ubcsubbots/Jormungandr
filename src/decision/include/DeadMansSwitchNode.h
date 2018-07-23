/*
 * Created By: Gareth Ellis
 * Created On: July 22nd, 2018
 * Description: A node that publishes how long it's been since it received a
 *              message on a given topic. This can be used by other nodes
 *              to check if they have died and how long ago. Currently it's
 *              used by the firmware running on the Arduino to check if the
 *              safety has previously been triggered (and so cut power to
 *              the Arduino).
 */

#ifndef DECISION_DEAD_MANS_SWITCH_NODE_H
#define DECISION_DEAD_MANS_SWITCH_NODE_H

// ROS Includes
#include <std_msgs/Empty.h>
#include <std_msgs/Duration.h>
#include <ros/ros.h>

class DeadMansSwitchNode {
  public:
    DeadMansSwitchNode(int argc, char** argv, std::string node_name);

  private:
    /**
     * Callback function for when a new string is received
     *
     * @param is_alive the string received in the callback
     */
    void isAliveCallback(const std_msgs::Empty::ConstPtr &is_alive);

    /**
     * Publishes how long it's been since we last received a msg
     */
    void publishTimeSinceLastMsgReceived(const ros::TimerEvent&);


    // A subscriber to the topic indicating if the node we're
    // checking is alive. It is the responsibility of that node
    // to publish this topic
    ros::Subscriber is_alive_subscriber;

    // Publishes how long it's been since we've received an
    // "is alive" message
    ros::Publisher time_since_last_alive_publisher;

    // The timer for regularly publishing how long it's been
    // since we received an "is alive" message
    ros::Timer time_since_last_alive_publisher_timer;

    // Whether or not we've received the first message
    bool first_message_received;

    // When the last time we received an "is alive" message was
    ros::Time last_time_msg_received;
};
#endif // DECISION_DEAD_MANS_SWITCH_NODE_H
