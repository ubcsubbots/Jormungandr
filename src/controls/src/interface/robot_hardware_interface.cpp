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
    // Declare and register shared memory handlers for imu sensor
    hardware_interface::ImuSensorHandle imu_handle("imu_sensor", "/",
        &imu_orientation_, &imu_orientation_covariance_,
        &imu_angular_velocity_, &imu_angular_velocity_covariance_,
        &imu_linear_acceleration_, &imu_linear_acceleration_covariance_);
    imu_sensor_interface_.registerHandle(imu_handle);

    // Register imu interface so it can be used by the imu controller
    registerInterface(&imu_sensor_interface_);

    // Declare and register shared memory handlers for thrusters 
    hardware_interface::ThrusterHandle thruster_one_handle("thruster_one", &thruster_cmd_[0]);
    thrusters_interface_.registerHandle(thruster_one_handle);

    hardware_interface::ThrusterHandle thruster_two_handle("thruster_two", &thruster_cmd_[1]);
    thrusters_interface_.registerHandle(thruster_two_handle);

    hardware_interface::ThrusterHandle thruster_three_handle("thruster_three", &thruster_cmd_[2]);
    thrusters_interface_.registerHandle(thruster_three_handle);

    hardware_interface::ThrusterHandle thruster_four_handle("thruster_four", &thruster_cmd_[3]);
    thrusters_interface_.registerHandle(thruster_four_handle);

    hardware_interface::ThrusterHandle thruster_five_handle("thruster_five", &thruster_cmd_[4]);
    thrusters_interface_.registerHandle(thruster_five_handle);

    hardware_interface::ThrusterHandle thruster_six_handle("thruster_six", &thruster_cmd_[5]);
    thrusters_interface_.registerHandle(thruster_six_handle);

    // register thruster interface so it can be used by the thruster controller
    registerInterface(&thrusters_interface_);

    // Declare and register shared memory handle for depth state
    hardware_interface::DepthStateHandle depth_state_handle("depth_sensor", &depth_);
    depth_state_interface_.registerHandle(depth_state_handle);

    // register depth state interface so it can be used by the depth state controller
    registerInterface(&depth_state_interface_);
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

