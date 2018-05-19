/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description: Abstract class for subroutines
 */
#ifndef DECISION_SUBROUTINE_H
#define DECISION_SUBROUTINE_H

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/**
 * pre defined directions for subroutines to use
 * example: z_rotation = RIGHT
 */
static const double RIGHT    = -1.0;
static const double LEFT     = 1.0;
static const double FORWARD  = 1.0;
static const double BACKWARD = -1.0;
static const double UP       = 1.0;
static const double DOWN     = -1.0;

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
