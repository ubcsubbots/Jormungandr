/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Thrusters driver
 */

#include "../include/ImuDriver.h"

namespace arduino_drivers
{
    ImuDriver::ImuDriver() {}
    ImuDriver::~ImuDriver() {}

    void ImuDriver::init() {}
    void ImuDriver::update(controls::DriversMsg* output_msg, const controls::DriversMsg* input_msg) {}
    
} // namespace robot_driver
