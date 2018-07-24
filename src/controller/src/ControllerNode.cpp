/*
 * Created By: Viral Galaiya
 * Created On: July 7th 2018
 * Description: The LQR controller with the integrator, determined using Simulink. Takes in the Ros geometry message and returns a//  ros custom message with the PWM
 */

#include "ControllerNode.h"

ControllerNode::ControllerNode(int argc, char** argv, std::string name) {

    ros::init(argc, argv, name);
    ros::NodeHandle nh;

    controller = Controller();

    // set this to the frequency of the controller
    ros::Rate loop_rate(1/controller.PERIOD);

    twist_sub = nh.subscribe("velocity_publisher", 1000, &ControllerNode::DesiredvelocityCallback, this);
    IMU_sub =  nh.subscribe("imu_data", 1000, &ControllerNode::IMUCallback, this);
    arduino_pub = nh.advertise<std_msgs::Int32MultiArray>("Arduino", 1000);
}

void setImuDataHelper(Controller &controller, geometry_msgs::Vector3 angular, geometry_msgs::Vector3 linear) {
    controller.setImuData(angular.x, angular.y, angular.z, linear.x, linear.y, linear.z);
}

void ControllerNode::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    setImuDataHelper(controller, msg->angular_velocity, msg->linear_acceleration);
}

Eigen::Matrix setDesiredVelocityHelper(Controller &controller, geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular, double position_z) {
    return controller.setDesiredVelocity(linear.x, linear.y, angular.x, angular.y, angular.z, position_z);
}

void ControllerNode::DesiredvelocityCallback(const nav_msgs::Odometry::ConstPtr &desired_twist_velocity) {
    Eigen::Matrix torque = setDesiredVelocityHelper(controller, desired_twist_velocity->twist.twist.linear, desired_twist_velocity->twist.twist.angular, desired_twist_velocity->pose.pose.position.z);

    std_msgs::Int32MultiArray motor_parameters;
    motor_parameters.data.push_back(torque(0,0));
    motor_parameters.data.push_back(torque(1,0));
    motor_parameters.data.push_back(torque(2,0));
    motor_parameters.data.push_back(torque(3,0));
    arduino_pub.publish(motor_parameters);
}