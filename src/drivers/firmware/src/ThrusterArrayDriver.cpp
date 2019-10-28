/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Thrusters driver
 */

#include "../include/ThrusterArrayDriver.h"

namespace arduino_drivers
{
    ThrusterArrayDriver::ThrusterArrayDriver() {}
    ThrusterArrayDriver::~ThrusterArrayDriver() {}

    void ThrusterArrayDriver::init() {}
    void ThrusterArrayDriver::update(controls::DriversMsg* output_msg, const controls::DriversMsg* input_msg) {}
    
} // namespace robot_driver