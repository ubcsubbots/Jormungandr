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

// Subbots Includes
#include <DeadMansSwitchNode.h>

DeadMansSwitchNode::DeadMansSwitchNode(int argc, char** argv, std::string node_name) :
first_message_received(false)
{
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get Params
    double time_since_last_alive_publish_rate_hz = 30;
    nh.getParam("time_since_last_alive_publish_rate_hz", time_since_last_alive_publish_rate_hz);

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "is_alive_input";
    uint32_t refresh_rate                  = 10;
    is_alive_subscriber                     = private_nh.subscribe(
    topic_to_subscribe_to, refresh_rate, &DeadMansSwitchNode::isAliveCallback, this);

    // Setup Publisher(s)
    std::string topic   = private_nh.resolveName("time_since_last_is_alive");
    uint32_t queue_size = 1;
    time_since_last_alive_publisher = private_nh.advertise<std_msgs::Duration>(topic, queue_size);

    // Setup a timer so we publish how long it's been since we
    // received an "is alive" message at regular intervals
    ros::Duration time_since_last_alive_publish_period( 1/ time_since_last_alive_publish_rate_hz);
    time_since_last_alive_publisher_timer  = private_nh.createTimer(
            time_since_last_alive_publish_period,
            &DeadMansSwitchNode::publishTimeSinceLastMsgReceived, this);
}

void DeadMansSwitchNode::isAliveCallback(const std_msgs::Empty::ConstPtr &is_alive) {
    first_message_received = true;
    last_time_msg_received = ros::Time::now();
}

void DeadMansSwitchNode::publishTimeSinceLastMsgReceived(const ros::TimerEvent &) {
    // Don't publish anything until we've received our first message
    if (!first_message_received) {
        return;
    }

    // Publish how long it's been since we last received a message
    ros::Time current_time = ros::Time::now();
    ros::Duration time_since_last_message = current_time - last_time_msg_received;

    std_msgs::Duration msg;
    msg.data = time_since_last_message;

    time_since_last_alive_publisher.publish(msg);
}
