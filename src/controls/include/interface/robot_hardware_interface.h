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
#include <interface/thruster_array_interface.h>
#include <interface/depth_state_interface.h>
#include <types/data_types.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <boost/shared_ptr.hpp>
#include <controls/DriversMsg.h>
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

    // Subscriber callback
    void driversCB(const controls::DriversMsg::ConstPtr& msg);

    // Provided interfaces
    hardware_interface::ThrusterArrayInterface thruster_array_interface_;

    ros::NodeHandle nh_;
    const static int msg_queue_ = 10;

    // Driver Subscriber
    ros::Subscriber drivers_sub_;

    // Realtime publishers to send messages to drivers in realtime
    typedef boost::shared_ptr<realtime_tools::RealtimePublisher<controls::DriversMsg> > RtPublisherPtr;
    RtPublisherPtr drivers_pub_;

    // Realtime buffer to recieve messages from drivers in realtime
    realtime_tools::RealtimeBuffer<DriversData> drivers_;

    // Memory allocated for subscriber messages
    DriversData drivers_struct_;

    // Shared memory
    ThrusterArrayData thruster_array_;
    ImuSensorData imu_sensor_;
    DepthSensorData depth_sensor_;
}; 

#endif //ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

