/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description:
 */
#ifndef DECISION_SUBROUTINE_H
#define DECISION_SUBROUTINE_H

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#define RIGHT -1.0
#define LEFT 1.0
#define FORWARD 1.0
#define BACKWARD -1.0
#define UP 1.0
#define DOWN -1.0

class Subroutine {
  public:
    Subroutine(int argc, char** argv, std::string node_name);

    void startup();
    void shutdown();

  protected:
    ros::Publisher publisher_;
    void publishCommand(const geometry_msgs::Twist& msg);
    geometry_msgs::Vector3 makeVector(double x, double y, double z);

    virtual void setupSubscriptions(ros::NodeHandle nh) = 0;
};

#endif // DECISION_SUBROUTINE_H
