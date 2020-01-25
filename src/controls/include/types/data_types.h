/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Data types for controllers
 */

#ifndef DATA__TYPES
#define DATA__TYPES

/**
 * Struct to store IMU data
 */
struct ImuSensorData
{

};

/**
 * Struct to store Depth Sensor data 
 */
struct DepthSensorData
{
    float depth;
};

/**
 * Struct to store Thruster array data
 */
struct ThrusterArrayData
{
    float thruster_one_command;
    float thruster_two_command;
    float thruster_three_command;
    float thruster_four_command;
    float thruster_five_command;
    float thruster_six_command;
};

/**
 * Struct to store data related to the drivers
 */
struct ArduinoDriversData
{
    DepthSensorData depth_sensor_data;
    ThrusterArrayData thruster_array_data;
};

#endif //DATA__TYPES
