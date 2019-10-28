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
    drivers_struct_= *(drivers_.readFromRT());
    // TODO: store the data from structs into the hardware interface's shared memory
}

void RobotHardwareInterface::write()
{
    controls::DriversMsg msg;
    // TODO: store thruster commands from shared memory into msg
    if (drivers_pub_ && drivers_pub_->trylock()){
            drivers_pub_->msg_ = msg;
            drivers_pub_->unlockAndPublish();
    }
}

void RobotHardwareInterface::initControllerInterfaces()
{
    // Declare and register shared memory handler for thruster array controller
    hardware_interface::ThrusterArrayHandle thruster_array_handle("thruster_array", 
        &thruster_array_,
        &imu_sensor_,
        &depth_sensor_);
    thruster_array_interface_.registerHandle(thruster_array_handle);

    // Register thruster array interface so it can be used by thruster array controller
    registerInterface(&thruster_array_interface_);
}

void RobotHardwareInterface::initDriverCommunication() 
{
    drivers_.initRT(drivers_struct_);

    drivers_sub_ = nh_.subscribe<controls::DriversMsg>(
        "/driver/output", msg_queue_,
        &RobotHardwareInterface::driversCB, this);

    drivers_pub_.reset(
        new realtime_tools::RealtimePublisher<controls::DriversMsg>(nh_,
            "/driver/input", 4));
}

void RobotHardwareInterface::driversCB(const controls::DriversMsg::ConstPtr& msg) 
{
    ROS_INFO("Got drivers message");
    DriversData data;
    // TODO: fill data with msg
    drivers_struct_= data;
    drivers_.writeFromNonRT(drivers_struct_);
}

