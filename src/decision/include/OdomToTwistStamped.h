/*
 * Created By: Cameron Newton
 * Created On: Sept 12nd, 2018
 * Description: A node that remaps the output of the decision node from an Odom
 * message to a TwistStamped for the simulator
 */

#ifndef JORMUNGANDR_ODOMTOTWISTSTAMPED_H
#define JORMUNGANDR_ODOMTOTWISTSTAMPED_H

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class OdomToTwistStamped {
  public:
    OdomToTwistStamped(int argc, char** argv, std::string node_name);

  private:
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    void odomMessageCallback(nav_msgs::Odometry odomMsg);
};

#endif // JORMUNGANDR_ODOMTOTWISTSTAMPED_H
