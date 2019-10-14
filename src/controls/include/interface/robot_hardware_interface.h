#ifndef ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <interface/thruster_command_interface.h>
#include <interface/depth_state_interface.h>

class RobotHardwareInterface:  public hardware_interface::RobotHW
{  
public:

    RobotHardwareInterface (int argc, char** argv, std::string node_name);
    ~RobotHardwareInterface ();
    void read();
    void write();
    
protected:

    // Provided interfaces
    hardware_interface::ImuSensorInterface imu_sensor_interface_;
    hardware_interface::ThrusterCommandInterface thrusters_interface_;
    hardware_interface::DepthStateInterface depth_state_interface_;

    // Shared memory
    double thruster_cmd_[6];
    double imu_orientation_;
    double imu_orientation_covariance_;
    double imu_angular_velocity_;
    double imu_angular_velocity_covariance_;
    double imu_linear_acceleration_;
    double imu_linear_acceleration_covariance_; 
    double depth_state_;  
            
}; 

#endif //ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

