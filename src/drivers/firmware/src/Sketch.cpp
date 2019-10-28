/*
 * Created By: Logan Fillo
 * Created On: October 26 2019
 * Description: Main Arduino sketch
 */

#include <ros.h>
#include <ArduinoHardware.h>
#include <controls/DriversMsg.h>

#include "../include/ThrusterArrayDriver.h"
#include "../include/DepthSensorDriver.h"
#include "../include/ImuDriver.h"

#include <Arduino.h>

/* Messages */
controls::DriversMsg output_msg_;
controls::DriversMsg input_msg_;

/* ROS variables setup */
void driversCB(const controls::DriversMsg& msg);
ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> nh_; // Custom nodehandle config
ros::Subscriber<controls::DriversMsg> drivers_sub_("/driver/input", &driversCB);
ros::Publisher drivers_pub_("/driver/output", &output_msg_ );

/* Drivers */
arduino_drivers::ImuDriver imu_driver_;
arduino_drivers::ThrusterArrayDriver multi_thruster_driver_;
arduino_drivers::DepthSensorDriver depth_sensor_driver_;

/**
 * Callback function for driver subscriber
 * 
 * @param msg The message containg input for the drivers
 */
void driversCB(const controls::DriversMsg& msg)
{ 
    input_msg_ = msg;
}

/**
 * Setup method run once on Arduino
 * 
 * Initalizes ROS node handle and all drivers 
 */
void setup() 
{
    nh_.initNode();
    nh_.advertise( drivers_pub_ );
    nh_.subscribe( drivers_sub_ );
    imu_driver_.init();
    multi_thruster_driver_.init();
    depth_sensor_driver_.init();
}

/**
 * Loop method run continously on Arduino
 * 
 * Updates all drivers and spins for callbacks, then sleeps
 */
void loop()
{
    imu_driver_.update( &output_msg_, &input_msg_ );
    depth_sensor_driver_.update( &output_msg_, &input_msg_ );
    multi_thruster_driver_.update( &output_msg_, &input_msg_ );
    drivers_pub_.publish ( &output_msg_ );
    nh_.spinOnce();
    delay(1000);
}