/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Depth Sensor Driver interface
 */

#ifndef DRIVERS__DEPTH_SENSOR_DRIVER_H
#define DRIVERS__DEPTH_SENSOR_DRIVER_H

#include "Driver.h"

namespace arduino_drivers
{
    class DepthSensorDriver : public Driver
    {
    public:
        DepthSensorDriver (/* args */);
        ~DepthSensorDriver ();

        void init();
        void update(controls::DriversMsg* output_msg, const controls::DriversMsg* input_msg);
       
    };
    
} // namespace arduino_drivers

#endif //DRIVERS__DEPTH_SENSOR_DRIVER_H