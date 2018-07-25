/*
 * Created By: Viral Galaiya
 * Created On: July 7th 2018
 * Description: The LQR controller with the integrator, determined using
 * Simulink. Takes in the Ros geometry message and returns a//  ros custom
 * message with the PWM
 */
#ifndef PROJECT_CONTROLLERNODE_H
#define PROJECT_CONTROLLERNODE_H

#include "Controller.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>

class ControllerNode {
  private:
    ros::Subscriber twist_subscriber_;
    ros::Subscriber depth_subscriber_;
    ros::Publisher arduino_publisher_;
    ros::Subscriber imu_subscriber_;

    Controller controller_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    void desiredVelocityCallback(
    const nav_msgs::Odometry::ConstPtr& desired_twist_velocity);

    void depthCallback( const std_msgs::Float32::ConstPtr& depth);

  public:
    ControllerNode(int argc, char** argv, std::string name);
};

#endif // PROJECT_CONTROLLERNODE_H
