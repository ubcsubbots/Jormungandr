/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Hardware interface header
 */

#ifndef ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <interface/thruster_array_interface.h>
#include <types/data_types.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <boost/shared_ptr.hpp>
#include <controls/ArduinoDriversMsg.h>
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
    void arduinoDriversCB(const controls::ArduinoDriversMsg::ConstPtr& msg);
    void imuDriverCB(const sensor_msgs::Imu::ConstPtr& msg);

    // Provided interfaces
    hardware_interface::ThrusterArrayInterface thruster_array_interface_;

    ros::NodeHandle nh_;
    const static int msg_queue_ = 10;

    // Driver Subscribers
    ros::Subscriber arduino_drivers_sub_;
    ros::Subscriber imu_driver_sub_;

    // Realtime publishers to send messages to drivers in realtime
    typedef boost::shared_ptr<realtime_tools::RealtimePublisher<controls::ArduinoDriversMsg> > RtPublisherPtr;
    RtPublisherPtr arduino_drivers_pub_;

    // Realtime buffer to recieve messages from drivers in realtime
    realtime_tools::RealtimeBuffer<ThrusterArrayData> thruster_array_;
    realtime_tools::RealtimeBuffer<DepthSensorData> depth_sensor_;
    realtime_tools::RealtimeBuffer<ImuSensorData> imu_sensor_;

    // Shared memory
    ThrusterArrayData thruster_array_struct_;
    ImuSensorData imu_sensor_struct_;
    DepthSensorData depth_sensor_struct_;
}; 

#endif //ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

