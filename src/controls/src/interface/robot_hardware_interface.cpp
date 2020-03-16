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
    auv_state_struct_ = *(auv_state_.readFromRT());

}

void RobotHardwareInterface::write()
{
    drivers_msgs::ArduinoDrivers msg;

    msg.thruster_array.thruster_one_command = thruster_array_struct_.thruster_one_command;
    msg.thruster_array.thruster_two_command = thruster_array_struct_.thruster_two_command;
    msg.thruster_array.thruster_three_command = thruster_array_struct_.thruster_three_command;
    msg.thruster_array.thruster_four_command = thruster_array_struct_.thruster_four_command;
    msg.thruster_array.thruster_five_command = thruster_array_struct_.thruster_five_command;
    msg.thruster_array.thruster_six_command = thruster_array_struct_.thruster_six_command;

    if (arduino_drivers_pub_ && arduino_drivers_pub_->trylock()){
            arduino_drivers_pub_->msg_ = msg;
            arduino_drivers_pub_->unlockAndPublish();
    }
}

void RobotHardwareInterface::initControllerInterfaces()
{
    // Declare and register shared memory handler for thruster array controller
    hardware_interface::ThrusterArrayHandle thruster_array_handle("thruster_array",
        &auv_state_struct_, 
        &thruster_array_struct_
    );
    thruster_array_interface_.registerHandle(thruster_array_handle);

    // Register thruster array interface so it can be used by thruster array controller
    registerInterface(&thruster_array_interface_);
}

void RobotHardwareInterface::initDriverCommunication() 
{
    // Initialize all structs to zero
    thruster_array_struct_ = {0};
    auv_state_struct_ = {0};

    thruster_array_.initRT(thruster_array_struct_);
    auv_state_.initRT(auv_state_struct_);

    arduino_drivers_pub_.reset(
        new realtime_tools::RealtimePublisher<drivers_msgs::ArduinoDrivers>(nh_,
            "/arduino_node/input", 4));
}



