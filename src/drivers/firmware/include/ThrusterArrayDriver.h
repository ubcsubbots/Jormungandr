/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Thrusters Driver interface
 */

#ifndef DRIVERS__THRSTERS_DRIVER_H
#define DRIVERS__THRSTERS_DRIVER_H

#include "Driver.h"

namespace arduino_drivers
{
    class ThrusterArrayDriver : public Driver
    {
    public:
        ThrusterArrayDriver (/* args */);
        ~ThrusterArrayDriver ();

        void init();
        void update(controls::DriversMsg* output_msg, const controls::DriversMsg* input_msg);
        
    };

} // namespace arduino_drivers

#endif //DRIVERS__THRSTERS_DRIVER_H