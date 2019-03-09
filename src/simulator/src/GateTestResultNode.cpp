/*
 * Created By: Logan Fillo
 * Created On: March 5th, 2019
 * Description: A node which subscribes to /uwsim/g500_odom (the simulation robot's pose and twist)
 *             and prints a string specifiying if the robot made it through the gate in a given
 *             amount of time.
 *
 */

#include <stdout>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

struct OdomMessage
{
  // Pose with covariance
  float pos_x;
  float pos_y;
  float pos_z;
  float quat_x;
  float quat_y;
  float quat_z;
  float quat_w
  float pose_cov[36];

  // Twist with covariance
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
  float twist_cov[36]

};

struct OdomMessage currMessage;

void odometryCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("[HEARD]: ODOMETRY_MESSAGE");

  // Setup currMessage
  currMessage.pos_x      = msg->pose.pose.position.x;
  currMessage.pos_y      = msg->pose.pose.position.y;
  currMessage.pos_z      = msg->pose.pose.position.z;
  currMessage.quat_x     = msg->pose.pose.orientation.x;
  currMessage.quat_y     = msg->pose.pose.orientation.y;
  currMessage.quat_z     = msg->pose.pose.orientation.z;
  currMessage.pose_cov   = msg->pose.covariance;

  currMessage.linear_x   = msg->twist.twist.linear.x;
  currMessage.linear_x   = msg->twist.twist.linear.y;
  currMessage.linear_x   = msg->twist.twist.linear.z;
  currMessage.angular_x  = msg->twist.twist.angular.x;
  currMessage.angular_x  = msg->twist.twist.angular.y;
  currMessage.angular_x  = msg->twist.twist.angular.z;
  currMessage.twist_cov  = msg->twist.covariance;
}

int main(int argc, char** argv)
{
    // Setup Nodehandle
    ros::init(argc, argv, "test result");
    ros::NodeHandle nh;

    // Setup subscriber
    std::string topic = "/uwsim/g500_odom";
    int refresh_rate  = 10;
    ros::Subscriber sim_sub = nh.subscribe(
      topic, refresh_rate, &odometryCallBack);

    ros::Rate loop_rate = 1;
    int count = 0;
    while (ros::ok()){
      // Check to see if it has gone through the gate
      if ()
        std::cout<<"PASSED"<<std::endl;

      if (count == 15) break;
      count ++;

    }
    std::cout<<"FAILED"<<std::endl;
}
