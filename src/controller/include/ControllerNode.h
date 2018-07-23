/*
 * Created By: Viral Galaiya
 * Created On: July 7th 2018
 * Description: The LQR controller with the integrator, determined using Simulink. Takes in the Ros geometry message and returns a//  ros custom message with the PWM
 */
#ifndef PROJECT_CONTROLLERNODE_H
#define PROJECT_CONTROLLERNODE_H

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "nav_msgs/Odometry.h"

#include <Eigen/Dense> //matrix manipulation library
#include "sensor_msgs/Imu.h"
#define PERIOD 0.02

class ControllerNode {
private:
    ros::Subscriber twist_sub;
    ros::Publisher arduino_pub;
    ros::Subscriber IMU_sub;

    //IMU data
    Eigen::MatrixXd IMUangularvelocity; //3x1
    Eigen::MatrixXd IMUangularaccelaration; //3x1

    Eigen::Vector3d linearvelocity;
    Eigen::MatrixXd IMUlinearaccelaration;

    Eigen::Vector3d previousIMUangularvelocity;
    Eigen::Vector3d previousIMUlinearaccelaration;

    //differentiation calculated

    Eigen::MatrixXd Desiredvelocity;
    Eigen::MatrixXd previousY;
    Eigen::MatrixXd previousDesiredvelocity;
    Eigen::MatrixXd intergratorAcumulator;

    Eigen::MatrixXd Currentvelocity;
    Eigen::MatrixXd Currentacceleration;
    Eigen::MatrixXd X;
    Eigen::MatrixXd K;//X
    Eigen::MatrixXd Ki;//ki*(cY-R)
    Eigen::MatrixXd Y;
    Eigen::MatrixXd torquematrix;
    Eigen::MatrixXd PWMmatrix;

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void DesiredvelocityCallback(const nav_msgs::Odometry::ConstPtr &desired_twist_velocity);

public:
    ControllerNode(int argc, char** argv, std::string name);
};

#endif //PROJECT_CONTROLLERNODE_H
