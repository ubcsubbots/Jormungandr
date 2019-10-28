/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Imu driver
 */

#ifndef DRIVERS__IMU_DRIVER_H
#define DRIVERS__IMU_DRIVER_H

#include "Driver.h"

namespace arduino_drivers
{
    class ImuDriver : public Driver
    {
    public:
        ImuDriver (/* args */);
        ~ImuDriver ();

        void init();
        void update(controls::DriversMsg* output_msg, const controls::DriversMsg* input_msg);
    };
    
} // namespace arduino_drivers


#endif //DRIVERS__IMU_DRIVER_H