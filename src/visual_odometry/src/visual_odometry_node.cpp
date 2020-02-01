/*
 * Created By: Andrew Chan
 * Created On: January 25, 2020
 * Description: A simple node 
 */
 
#include <ros/ros.h>
#include "visual_odometry_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_odometry_node");

    std::string node_name = "visual_odometry_node";
    ros::NodeHandle nh;

    ROS_INFO("Hello, World!");

    ros::spin();
  
    return 0;
}


