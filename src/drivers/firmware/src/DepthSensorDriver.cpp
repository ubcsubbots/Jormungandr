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

    void DepthSensorDriver::init() {}
    void DepthSensorDriver::update(controls::DriversMsg* output_msg, const controls::DriversMsg* input_msg) {}
    
} // namespace robot_driver