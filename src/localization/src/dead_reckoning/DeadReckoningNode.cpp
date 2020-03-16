#include <dead_reckoning/DeadReckoningNode.h>

DeadReckoningNode::DeadReckoningNode(int argc, char** argv)
{   
    std::string arduino_drivers_topic = "/arduino_drivers/output";
    std::string imu_driver_topic = "/imu/data";
    
    arduino_drivers_sub = nh.subscribe(arduino_drivers_topic, queue_size, 
                                        &DeadReckoningNode::arduinoDriversCallback, this);

    imu_driver_sub = nh.subscribe(imu_driver_topic, queue_size,
                                &DeadReckoningNode::imuDriverCallback, this);
}

void DeadReckoningNode::arduinoDriversCallback(const drivers::ArduinoDrivers::ConstPtr& msg) 
{
    // ROS_INFO("Got Arduino drivers message");  
    float depth = msg->depth_sensor.depth;

}

void DeadReckoningNode::imuDriverCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ROS_INFO("Got IMU driver message");

    // Convert quat orientation to euler angles (phi theta psi)
    tf::Quaternion quat( msg->orientation.x, 
                          msg->orientation.y, 
                          msg->orientation.z, 
                          msg->orientation.w);

    tf::Matrix3x3 mat(quat);

    double phi, theta, psi;

    mat.getRPY(phi, theta, psi);

}