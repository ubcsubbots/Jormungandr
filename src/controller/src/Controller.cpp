/*
 * Created By: Viral Galaiya
 * Created On: July 23, 2018
 * Description:
 */
#include "Controller.h"

Controller::Controller() {
    previousIMUangularvelocity  <<0,0,0;
    linearvelocity                <<0,0,0;
    previousIMUlinearaccelaration <<0,0,0;
    previousY                     <<0,0,0,0,0,0;
    Desiredvelocity         <<0,0,0,0,0,0;
    previousY                     <<0,0,0,0,0,0;
    previousDesiredvelocity <<0,0,0,0,0,0;
    intergratorAcumulator         <<0,0,0,0,0,0;
    Ki <<
       -5.0075 ,  4.6995 ,   0.0742 , -0.0000  ,  0.0000  , -1.6221,
            -4.1898 , -5.8310 ,  -0.0892 ,  0.0000  , -0.0000  ,  1.7134,
            0.1424  ,-2.3458  , -1.6064  ,-0.0000   , -0.0000  , -3.2109,
            -0.1494 ,  3.0911 ,  -1.5075 , -0.0000  ,  0.0000  ,  3.2398;

    K <<
      0.1138  ,-0.1855  , -0.0318   ,-0.1248  ,  0.0023  ,  0.1622  ,  1.5326  , -0.7185   ,-0.0201  ,  0.2112  ,  0.0016   , 0.1854   ,
            0.0952  , 0.2302  ,  0.0382   , 0.1272  , -0.0020  , -0.1713  ,  1.5493  ,  0.7596   , 0.0245  , -0.2109  ,  0.0015   ,-0.1943   ,
            -0.0032 ,  0.0926 ,   0.6885  ,  0.1288 ,  -0.0127 ,   0.3211 ,  -0.0460 ,   0.8994  ,  0.3173 ,  -0.0805 ,   0.0456  ,  0.3836  ,
            0.0034  ,-0.1220  ,  0.6461   , 0.8552  , -0.0102  , -0.3240  ,  0.0473  , -1.0260   , 0.4615  ,  0.0795  ,  0.0456   ,-0.3864 ;

}

void Controller::setImuData(double angular_x, double angular_y, double angular_z, double linear_x, double linear_y,
                            double linear_z) {
    IMUangularvelocity << angular_x, angular_y, angular_z;
    IMUlinearaccelaration << linear_x, linear_y, linear_z;
}

Eigen::Matrix Controller::setDesiredVelocity(double linear_x, double linear_y, double angular_x, double angular_y,
                                    double angular_z, double position_z) {
    IMUangularaccelaration = (IMUangularvelocity - IMUangularvelocity)/PERIOD;

    Desiredvelocity << linear_x, linear_y, position_z, angular_x, angular_y, angular_z;

    // Actual velocity to be got from the IMU and depth to be got from the pressure/depth sensor
    linearvelocity = linearvelocity + (IMUlinearaccelaration + previousIMUlinearaccelaration) * PERIOD * 0.5; //account for cummilation error and add reset

    Currentvelocity <<
                    IMUangularvelocity.coeff(0,0),
            IMUangularvelocity.coeff(1,0),
            IMUangularvelocity.coeff(2,0),//position, not velocity
            linearvelocity.coeff(0,0),
            linearvelocity.coeff(1,0),
            linearvelocity.coeff(2,0);

    Currentacceleration <<
                        IMUangularaccelaration.coeff(0,0),
            IMUangularaccelaration.coeff(1,0),
            IMUangularaccelaration.coeff(2,0), //to be changed from vel to osition
            IMUlinearaccelaration.coeff(0,0),
            IMUlinearaccelaration.coeff(1,0),
            IMUlinearaccelaration.coeff(2,0);

    ;// u = −K(x − r) − Ki*Y where r is Desiredvelocity  and Y is desired velocity
    // u place holder


    //parameters used to get the K from LQI in MATLAB
    //Q= diag([3 3 3 3 3 3 3 3 3 3 3 3 44 76 7 3 3 30]*1); may need to change, to heavy a weight on feedback and error on the intergrator (may?) be super high
    //R = diag([1 1 1 1]);
    //N = eye(18,4)*1;

    // PWM matrix needs to be multiplied to get the PWM value, since its in terms of torque rightnow

    Y <<
      Currentvelocity(0,0),
            Currentvelocity(1,0),
            Currentvelocity(2,0),
            Currentvelocity(3,0),
            Currentvelocity(4,0),
            Currentvelocity(5,0),
            X <<
              Currentvelocity(0,0),
            Currentvelocity(1,0),
            Currentvelocity(2,0),
            Currentvelocity(3,0),
            Currentvelocity(4,0),
            Currentvelocity(5,0),
            Currentacceleration(0,0),
            Currentacceleration(1,0),
            Currentacceleration(2,0),
            Currentacceleration(3,0),
            Currentacceleration(4,0),
            Currentacceleration(5,0);

    intergratorAcumulator = intergratorAcumulator + ((Desiredvelocity + previousDesiredvelocity)/2  - (Y+previousY)/2)*PERIOD;

    torquematrix = -Ki * intergratorAcumulator  +  -K * X;
    //T100 equation (for <0 thrust) 1454 +298*x+56.7*x^2
    //T100 equation (for >0 thrust) 1543+179*-16.5*x^2
    //T200 equation (for <0 thrust) 1457+104*x+5.1+x^2
    //T200 equation (for >0 thrust) 1539+89.9*x-3.91*x^2


    //c convert to torque
    if(torquematrix(0,0) < 0)
        PWMmatrix(0,0) = 1454 + 298* torquematrix(0,0) + 56.7 * torquematrix(0,0) * torquematrix(0,0);
    else if(torquematrix(1,0) > 0)
        PWMmatrix(0,0) = 1543 + 179* torquematrix(0,0) - 16.5 * torquematrix(0,0) *torquematrix(0,0);
    else
        PWMmatrix(1,0) = 0;

    if(torquematrix(1,0) < 0)
        PWMmatrix(1,0) = 1454 + 298* torquematrix(1,0) + 56.7 * torquematrix(1,0) * torquematrix(1,0);
    else if(torquematrix(1,0) > 0)
        PWMmatrix(1,0) = 1543 + 179* torquematrix(1,0) - 16.5 * torquematrix(1,0) * torquematrix(1,0);
    else
        PWMmatrix(1,0) = 0;

    if(torquematrix(2,0) < 0)
        PWMmatrix(2,0) = 1454 + 104* torquematrix(2,0) + 5.1 *  torquematrix(2,0) * torquematrix(2,0);
    else if(torquematrix(2,0) > 0)
        PWMmatrix(2,0) = 539 + 89.9* torquematrix(2,0) - 3.91 * torquematrix(2,0) * torquematrix(2,0);
    else
        PWMmatrix(2,0) = 0;

    if(torquematrix(3,0) < 0)
        PWMmatrix(3,0) = 1454 + 104* torquematrix(3,0) + 5.1 *  torquematrix(3,0) * torquematrix(3,0);
    else if(torquematrix(1,0) > 0)
        PWMmatrix(3,0) = 539 + 89.9* torquematrix(3,0) - 3.91 * torquematrix(3,0) * torquematrix(3,0);
    else
        PWMmatrix(2,0) = 0;

    previousIMUlinearaccelaration = IMUlinearaccelaration;
    previousIMUangularvelocity = IMUangularvelocity;
    Desiredvelocity = previousDesiredvelocity;
    previousY =Y;

    return torquematrix;
}