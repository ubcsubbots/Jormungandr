/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Hardware interface
 */

#include <interface/robot_hardware_interface.h>

RobotHardwareInterface::RobotHardwareInterface(int argc, char** argv, std::string node_name)
{
    initControllerInterfaces();
    initDriverCommunication();
}

RobotHardwareInterface::~RobotHardwareInterface() {}

void RobotHardwareInterface::read()
{
    thruster_array_struct_= *(thruster_array_.readFromRT());
    depth_sensor_struct_= *(depth_sensor_.readFromRT());
    imu_sensor_struct_= *(imu_sensor_.readFromRT());

}

void RobotHardwareInterface::write()
{
    controls::ArduinoDriversMsg msg;
    // TODO: store thruster commands from shared memory into msg
    if (arduino_drivers_pub_ && arduino_drivers_pub_->trylock()){
            arduino_drivers_pub_->msg_ = msg;
            arduino_drivers_pub_->unlockAndPublish();
    }
}

void RobotHardwareInterface::initControllerInterfaces()
{
    // Declare and register shared memory handler for thruster array controller
    hardware_interface::ThrusterArrayHandle thruster_array_handle("thruster_array", 
        &thruster_array_struct_,
        &imu_sensor_struct_,
        &depth_sensor_struct_);
    thruster_array_interface_.registerHandle(thruster_array_handle);

    // Register thruster array interface so it can be used by thruster array controller
    registerInterface(&thruster_array_interface_);
}

void RobotHardwareInterface::initDriverCommunication() 
{
    thruster_array_.initRT(thruster_array_struct_);
    depth_sensor_.initRT(depth_sensor_struct_);

    arduino_drivers_sub_ = nh_.subscribe<controls::ArduinoDriversMsg>(
        "/arduino_drivers_node/output", msg_queue_,
        &RobotHardwareInterface::arduinoDriversCB, this);

    arduino_drivers_pub_.reset(
        new realtime_tools::RealtimePublisher<controls::ArduinoDriversMsg>(nh_,
            "/arduino_drivers_node/input", 4));
    
    imu_driver_sub_ = nh_.subscribe<sensor_msgs::Imu>(
        "/imu/data", msg_queue_,
        &RobotHardwareInterface::imuDriverCB, this);
}

void RobotHardwareInterface::arduinoDriversCB(const controls::ArduinoDriversMsg::ConstPtr& msg) 
{
    ROS_INFO("Got Arduino drivers message");

    depth_sensor_struct_.depth = msg->depth_sensor.depth;

    thruster_array_struct_.thruster_one_command = msg->thruster_array.thruster_one_command;
    thruster_array_struct_.thruster_two_command = msg->thruster_array.thruster_two_command;
    thruster_array_struct_.thruster_three_command = msg->thruster_array.thruster_three_command;
    thruster_array_struct_.thruster_four_command = msg->thruster_array.thruster_four_command;
    thruster_array_struct_.thruster_five_command = msg->thruster_array.thruster_five_command;
    thruster_array_struct_.thruster_six_command = msg->thruster_array.thruster_six_command;

    // ROS_INFO("Depth: %f", depth_sensor_struct_.depth);

    thruster_array_.writeFromNonRT(thruster_array_struct_);
    depth_sensor_.writeFromNonRT(depth_sensor_struct_);
}

void RobotHardwareInterface::imuDriverCB(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_INFO("Got IMU drivers message");

    // TODO: fill imu struct here

    imu_sensor_.writeFromNonRT(imu_sensor_struct_);

}

