/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Depth sensor driver
 */

#include "../include/DepthSensorDriver.h"

namespace arduino_drivers
{
    DepthSensorDriver::DepthSensorDriver() {}
    DepthSensorDriver::~DepthSensorDriver() {}

    void DepthSensorDriver::init() 
    {
        while (!sensor.init()) 
        {
            delay(5000);
        }

    }
    void DepthSensorDriver::update(controls::DriversMsg* output_msg, const controls::DriversMsg* input_msg) 
    {
        sensor.read();
        float depth = sensor.depth();
    }
    
} // namespace robot_driver