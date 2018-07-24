/*
 * Created By: Viral Galaiya
 * Created On: July 7th 2018
 * Description: The LQR controller with the integrator, determined using Simulink. Takes in the Ros geometry message and returns a//  ros custom message with the PWM
 */
#ifndef PROJECT_CONTROLLERNODE_H
#define PROJECT_CONTROLLERNODE_H

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "Controller.h"


class ControllerNode {
private:
    ros::Subscriber twist_sub;
    ros::Publisher arduino_pub;
    ros::Subscriber IMU_sub;

    Controller controller;

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void DesiredvelocityCallback(const nav_msgs::Odometry::ConstPtr &desired_twist_velocity);

public:
    ControllerNode(int argc, char** argv, std::string name);
};

#endif //PROJECT_CONTROLLERNODE_H
