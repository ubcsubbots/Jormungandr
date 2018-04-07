/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description:
 */
#ifndef DECISION_SUBROUTINE_H
#define DECISION_SUBROUTINE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Subroutine {
public:
    Subroutine(int argc, char **argv, std::string node_name);

    void startup();
    void shutdown();

protected:
    ros::Publisher publisher_;
    void publishCommand(const geometry_msgs::Twist &msg);
    geometry_msgs::Vector3 makeVector(double x, double y, double z);

    virtual void setupSubscriptions(ros::NodeHandle nh) = 0;
};

#endif //DECISION_SUBROUTINE_H
