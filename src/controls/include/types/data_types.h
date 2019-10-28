/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Data types for controllers
 */

#ifndef DATA__TYPES
#define DATA__TYPES

/**
 * Struct to store Imu data
 */
struct ImuSensorData
{
    // TODO: store required data here
};

/**
 * Struct to store Depth Sensor data 
 */
struct DepthSensorData
{
    // TODO: store required data here
};

/**
 * Struct to store Thruster array data
 */
struct ThrusterArrayData
{
    // TODO: store required data here
};

/**
 * Struct to store data related to the drivers
 */
struct DriversData
{
    ImuSensorData imu_data;
    DepthSensorData depth_sensor_data;
    ThrusterArrayData thruster_array_data;
};

#endif //DATA__TYPES
