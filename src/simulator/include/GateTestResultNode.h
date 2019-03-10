/*
 * Created By: Logan Fillo
 * Created On: March 9th, 2019
 * Description: Checks incoming data from simulation robot (girona500)
 *              and determines whether or not it has made it through
 *              the  gate within a specified amount of time
 *
 */

#ifndef JORMUNGANDR_GATETESTRESULTNODE_H
#define JORMUNGANDR_GATETESTRESULTNODE_H

#include <stdout>
#include <ros/ros.h>

/* ROS msg types */
#include <nav_msgs/Odometry.h>

class GateTestResultNode {
  public:
    GateTestResultNode(int argc, char** argv, std::string node_name);

  private:
    nav_msgs::Odometry::ConstPtr& currMessage;
    ros::Subscriber sim_robot_odom;

    /**
     * Callback function for when data is received from girona500
     *
     * @param odometry discretized messages
     */
    void odometryCallBack(const nav_msgs::Odometry::ConstPtr& msg);


};

 #endif //JORMUNGANDR_GATETESTRESULTNODE_H
