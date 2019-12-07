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

    void ThrusterArrayDriver::init() 
    {
        // Attach servo objects to respective pins
        servo1.attach(servoPin1);
        servo2.attach(servoPin2);
        servo3.attach(servoPin3);
        servo4.attach(servoPin4);
        servo5.attach(servoPin5);
        servo6.attach(servoPin6);


        // Send "stop" signal to all ESCs.
        servo1.writeMicroseconds(1500+25, 1); 
        servo2.writeMicroseconds(1500+25, 1); 
        servo3.writeMicroseconds(1500+25, 1); 
        servo4.writeMicroseconds(1500+25, 1); 
        servo5.writeMicroseconds(1500+25, 1); 
        servo6.writeMicroseconds(1500+25, 1); 

        // delay to allow the ESC to recognize the stopped signal
        delay(10000); 

    }
    void ThrusterArrayDriver::update(controls::DriversMsg* output_msg, const controls::DriversMsg* input_msg) 
    {
        int fudge_factor=25;
        int signal = 1600 + fudge_factor;
        bool invert=1;

        // Send signal to ESC.
        servo1.writeMicroseconds(signal,invert); 
        servo2.writeMicroseconds(signal,invert); 
        servo3.writeMicroseconds(signal,invert); 
        servo4.writeMicroseconds(signal,invert); 
        servo5.writeMicroseconds(signal,invert); 
        servo6.writeMicroseconds(signal,invert); 

    }
    
} // namespace robot_driver