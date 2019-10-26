/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Hardware interface header
 */

#ifndef ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <interface/thruster_command_interface.h>
#include <interface/depth_state_interface.h>
#include <drivers/ImuDriver.h>
#include <drivers/ThrustersDriver.h>
#include <drivers/DepthSensorDriver.h>
#include <DataStructs.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <boost/shared_ptr.hpp>
#include <controls/MultiThrusterCommand.h>
#include <controls/DepthSensorState.h>
#include <sensor_msgs/Imu.h>

class RobotHardwareInterface:  public hardware_interface::RobotHW
{  
public:

    RobotHardwareInterface (int argc, char** argv, std::string node_name);
    ~RobotHardwareInterface ();

    /**
     * Reads the current state of each driver (as nessesary) into the shared 
     * memory to be used by the handlers in each controller.
     */
    void read();

    /**
     * Writes the current state of the shared memory to each driver (as nessesary) 
     */
    void write();
    
protected:

    /**
     * Initializes the robot hardware interface
     * 
     * Sets up handles for controllers to use and registers the associated 
     * interfaces for each set of handles.
     */
    void initControllerInterfaces(); 

    /**
     * Initializes the communication with drivers
     * 
     * Sets up realtime publishers and buffers for each controller
     * as nessesary to allow for realtime communcation between the
     * hardware interface and drivers
     */
    void initDriverCommunication(); 

    // Subscriber callbacks 
    void imuCB(const sensor_msgs::Imu::ConstPtr& msg);
    void multiThrusterCB(const controls::MultiThrusterCommand::ConstPtr& msg);
    void depthSensorCB(const controls::DepthSensorState::ConstPtr& msg);

    // Provided interfaces
    hardware_interface::ImuSensorInterface imu_sensor_interface_;
    hardware_interface::ThrusterCommandInterface thrusters_interface_;
    hardware_interface::DepthStateInterface depth_state_interface_;

    // Driver Subscribers
    ros::Subscriber imu_driver_sub_;
    ros::Subscriber multi_thruster_driver_sub_;
    ros::Subscriber depth_sensor_driver_sub_;

    ros::NodeHandle nh_;
    const static int msg_queue_ = 10;

    // Realtime publishers to send messagses t drivers in realtime
    typedef boost::shared_ptr<realtime_tools::RealtimePublisher<controls::MultiThrusterCommand> > RtPublisherPtr;
    RtPublisherPtr multi_thruster_driver_pub_;

    // Realtime buffers to recieve messages from drivers in realtime
    realtime_tools::RealtimeBuffer<ImuData> imu_driver_;
    realtime_tools::RealtimeBuffer<MultiThrusterData> multi_thruster_driver_ ;
    realtime_tools::RealtimeBuffer<DepthSensorData>  depth_sensor_driver_;

    // Memory allocated for subscriber messages
    ImuData imu_driver_struct_;
    MultiThrusterData multi_thruster_driver_struct_;
    DepthSensorData depth_sensor_driver_struct_;

    // Shared memory
    double thruster_cmd_[6];
    double imu_orientation_;
    double imu_orientation_covariance_;
    double imu_angular_velocity_;
    double imu_angular_velocity_covariance_;
    double imu_linear_acceleration_;
    double imu_linear_acceleration_covariance_; 
    double depth_;  
}; 

#endif //ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

