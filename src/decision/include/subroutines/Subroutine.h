/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description: Abstract class for subroutines
 */

#ifndef DECISION_SUBROUTINE_H
#define DECISION_SUBROUTINE_H

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

/**
 * pre defined directions for subroutines to use
 * example: z_rotation = RIGHT
 */
<<<<<<< HEAD
static const double RIGHT    = -0.15;
static const double LEFT     = 0.15;
static const double FORWARD  = 0.15;
static const double BACKWARD = -0.15;
static const double UP       = 0.15;
static const double DOWN     = -0.15;
=======
static const double RIGHT    = -0.25;
static const double LEFT     = 0.25;
static const double FORWARD  = 0.25;
static const double BACKWARD = -0.25;
static const double UP       = 0.25;
static const double DOWN     = -0.25;
>>>>>>> 7429af0abb1b30dc44f7656dc096d96b18c38ff2

class Subroutine {
  public:
    Subroutine(int argc, char** argv, std::string node_name);

    void startup();
    void shutdown();

    virtual void sleep() = 0;

  protected:
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    void publishCommand(const geometry_msgs::TwistStamped& msg);
    geometry_msgs::Vector3 makeVector(double x, double y, double z);

    virtual void setupSubscriptions(ros::NodeHandle nh) = 0;
};

#endif // DECISION_SUBROUTINE_H
