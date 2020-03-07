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
        &thruster_array_struct_,
        &imu_sensor_struct_,
        &depth_sensor_struct_);
    thruster_array_interface_.registerHandle(thruster_array_handle);

    // Register thruster array interface so it can be used by thruster array controller
    registerInterface(&thruster_array_interface_);
}

void RobotHardwareInterface::initDriverCommunication() 
{
    // Initialize all structs to zero
    thruster_array_struct_ = {0};
    depth_sensor_struct_ = {0};
    imu_sensor_struct_ = {0};

    thruster_array_.initRT(thruster_array_struct_);
    depth_sensor_.initRT(depth_sensor_struct_);

    arduino_drivers_sub_ = nh_.subscribe<controls::ArduinoDriversMsg>(
        "/arduino_node/output", msg_queue_,
        &RobotHardwareInterface::arduinoDriversCB, this);

    arduino_drivers_pub_.reset(
        new realtime_tools::RealtimePublisher<controls::ArduinoDriversMsg>(nh_,
            "/arduino_node/input", 4));
    
    imu_driver_sub_ = nh_.subscribe<sensor_msgs::Imu>(
        "/imu/data", msg_queue_,
        &RobotHardwareInterface::imuDriverCB, this);
}

void RobotHardwareInterface::arduinoDriversCB(const controls::ArduinoDriversMsg::ConstPtr& msg) 
{
    // ROS_INFO("Got Arduino drivers message");

    depth_sensor_struct_.depth = msg->depth_sensor.depth;

    thruster_array_struct_.thruster_one_command = msg->thruster_array.thruster_one_command;
    thruster_array_struct_.thruster_two_command = msg->thruster_array.thruster_two_command;
    thruster_array_struct_.thruster_three_command = msg->thruster_array.thruster_three_command;
    thruster_array_struct_.thruster_four_command = msg->thruster_array.thruster_four_command;
    thruster_array_struct_.thruster_five_command = msg->thruster_array.thruster_five_command;
    thruster_array_struct_.thruster_six_command = msg->thruster_array.thruster_six_command;

    thruster_array_.writeFromNonRT(thruster_array_struct_);
    depth_sensor_.writeFromNonRT(depth_sensor_struct_);
}

void RobotHardwareInterface::imuDriverCB(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ROS_INFO("Got IMU drivers message");

    imu_sensor_struct_.x_accel = msg->linear_acceleration.x;
    imu_sensor_struct_.y_accel = msg->linear_acceleration.y;
    imu_sensor_struct_.z_accel = msg->linear_acceleration.z;

    imu_sensor_struct_.x_ang_vel = msg->angular_velocity.x;
    imu_sensor_struct_.y_ang_vel = msg->angular_velocity.y;
    imu_sensor_struct_.z_ang_vel = msg->angular_velocity.z;

    // Convert quat orientation to euler angles (phi theta psi)
    tf::Quaternion quat( msg->orientation.x, 
                          msg->orientation.y, 
                          msg->orientation.z, 
                          msg->orientation.w);

    tf::Matrix3x3 mat(quat);

    double phi, theta, psi;

    mat.getRPY(phi, theta, psi);

    imu_sensor_struct_.phi = phi;
    imu_sensor_struct_.theta = theta;
    imu_sensor_struct_.psi = psi;

    imu_sensor_.writeFromNonRT(imu_sensor_struct_);

}

