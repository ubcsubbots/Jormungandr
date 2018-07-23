/*
 * Created By: Viral Galaiya
 * Created On: July 7th 2018
 * Description: The LQR controller with the integrator, determined using Simulink. Takes in the Ros geometry message and returns a//  ros custom message with the PWM
 */

#include "ControllerNode.h"

ControllerNode::ControllerNode(int argc, char** argv, std::string name) {

    ros::init(argc, argv, name);
    ros::NodeHandle nh;
    //constructor
    //initialize variables

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





    //Can the same node subscribe and publish?
    twist_sub = nh.subscribe("velocity_publisher", 1000, &ControllerNode::DesiredvelocityCallback, this);
    IMU_sub =  nh.subscribe("imu_data", 1000, &ControllerNode::IMUCallback, this);
    arduino_pub = nh.advertise<std_msgs::Int32MultiArray>("Arduino",1000);


}

void ControllerNode::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg) {

    IMUangularvelocity <<
                       msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;
    IMUlinearaccelaration <<
                          msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z;
}

void ControllerNode::DesiredvelocityCallback(const nav_msgs::Odometry::ConstPtr &desired_twist_velocity) {


    IMUangularaccelaration = (IMUangularvelocity - IMUangularvelocity)/PERIOD;

    Desiredvelocity <<
                    desired_twist_velocity->twist.twist.linear.x,
            desired_twist_velocity->twist.twist.linear.y,
            desired_twist_velocity->pose.pose.position.z,//position in real, not velocity
            desired_twist_velocity->twist.twist.angular.x,
            desired_twist_velocity->twist.twist.angular.y,
            desired_twist_velocity->twist.twist.angular.z;


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




    // set this to the frequency of the controller
    ros::Rate loop_rate(1/PERIOD);


    std_msgs::Int32MultiArray motor_parameters;
    motor_parameters.data.clear();
    motor_parameters.data.push_back(torquematrix(0,0));
    motor_parameters.data.push_back(torquematrix(1,0));
    motor_parameters.data.push_back(torquematrix(2,0));
    motor_parameters.data.push_back(torquematrix(3,0));
    arduino_pub.publish(motor_parameters);


    previousIMUlinearaccelaration = IMUlinearaccelaration;
    previousIMUangularvelocity = IMUangularvelocity;
    Desiredvelocity = previousDesiredvelocity;
    previousY =Y;
}