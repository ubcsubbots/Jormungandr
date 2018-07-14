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
static const double RIGHT    = -0.25;
static const double LEFT     = 0.25;
static const double FORWARD  = 0.25;
static const double BACKWARD = -0.25;
static const double UP       = 0.25;
static const double DOWN     = -0.25;

class Subroutine {
  public:
    Subroutine();

    void startup();
    void shutdown();

  protected:
    ros::Publisher publisher_;

    // these are used to set up and tear down routine specific subs/pubs
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    void publishCommand(const geometry_msgs::Twist& msg);
    geometry_msgs::Vector3 makeVector(double x, double y, double z);

    virtual void setupSubscriptions(ros::NodeHandle nh) = 0;
};

#endif // DECISION_SUBROUTINE_H
