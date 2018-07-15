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
    virtual std::string getName() = 0;

    void startup();
    void shutdown();

  protected:
    // hold onto these, automatically unsubscribe/unadvertise when out of scope
    ros::Publisher publisher_;
    std::vector<ros::Subscriber> subscriptions_;

    void publishCommand(const geometry_msgs::Twist& msg);
    geometry_msgs::Vector3 makeVector(double x, double y, double z);

    virtual void setupSubscriptions(ros::NodeHandle nh) = 0;
};

#endif // DECISION_SUBROUTINE_H
