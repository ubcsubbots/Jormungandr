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
struct ImuData
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
 * Struct to store Thrusters data
 */
struct MultiThrusterData
{
    // TODO: store required data here
};

struct DriversData
{
    ImuData imu_data;
    DepthSensorData depth_sensor_data;
    MultiThrusterData multi_thruster_data;
};

#endif //DATA__TYPES
