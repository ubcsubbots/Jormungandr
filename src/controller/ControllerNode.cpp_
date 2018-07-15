/*
 * Created By: Viral Galaiya
 * Created On: July 7th 2018
 * Description: The LQR controller with the intergrator, dertmined using Simulink. Takes in the Ros geometry message and returns a//  ros cutom message with the PWM
 */
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_mgs/Int16MultiArray.h"
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense> //matrix manipulation labrary

//void LQR(const geometry_msgs::Twist& velocity){

//}


void DesiredVelocityCallback(const geometry_msgs::Twist& desired_twist_velocity)
{
  Eigen::MatrixXd MatrixDesiredvelocity(6,1);
  MatrixDesiredvelocity <<
    twist_velocity->linear.x,
    twist_velocity->linear.y,
    twist_velocity->linear.z,
    twist_velocity->angular.x,
    twist_velocity->angular.y,
    twist_velocity->angular.z;

  Eigen::MatrixXd MatrixCurrentvelocity(6,1);
  MatrixDesiredvelocity <<
    1,
    2,
    3,
    4,
    5,
    6;

  // u = −K(x − xe) − Kiz + ud
  // u place holder
  Eigen::MatrixXd K(4,6);

  K <<
    1, 2, 3, 4, 5, 6,
    1, 2, 3, 4, 5, 6,
    1, 2, 3, 4, 5, 6,
    1, 2, 3, 4, 5, 6;
 
  Eigen::MatrixXd PWMmatrix(4,1);
  PWMmatrix = K(MatrixDesiredvelocity- MatrixCurrentvelocity);
  ros::Rate loop_rate(10);


  ros::Publisher arduino_pub = nh.advertise<std_msgs::Int16MultiArray>("Arduino",1000);

  std_msgs::Int16MultiArray motor_parameters;
  motor_parameters.data.clear();
  int T100port = 100; //temp
  int T100starboard = 100; //temp 
  int T200left = 200; //temp
  int T200right = 200;
  motor_parameters.data.pushback(PWMmatrix(1,1));
  motor_parameters.data.pushback(PWMmatrix(2,1));
  motor_parameters.data.pushback(PWMmatrix(3,1));
  motor_parameters.data.pushback(PWMmatrix(4,1));
  arduino_pub.publish(msg);


  
}


int main(int argc, char **argv){

  //creates the node name something to do with command line
  ros::init(argc, argv, "motor_controller_node");


  ros::spin();    
    return 0;

}


