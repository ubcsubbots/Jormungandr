/*
 * Created By: Viral Galaiya
 * Created On: July 7th 2018
 * Description: The LQR controller with the integrator, determined using Simulink. Takes in the Ros geometry message and returns a//  ros custom message with the PWM
 */
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "nav_msgs/Odometry.h"

#include <Eigen/Dense> //matrix manipulation library
#include "sensor_msgs/Imu.h"


class ControllerNode {
private:
  ros::Subscriber twist_sub;
  ros:: Publisher  arduino_pub;
  ros::Subscriber IMU_sub;
  ros::NodeHandle nh;


public:
  Eigen::MatrixXd IMUangularvelocity;
  Eigen::MatrixXd IMUlinearaccelaration;
  Eigen::MatrixXd cumilativeIMUangularvelocity;
  Eigen::MatrixXd cumilativeIMUlinearaccelaration;
  Eigen::MatrixXd previousIMUlinearaccelaration;
  ControllerNode (std::string name){
    //initializes node

    //should be custom twist??..publishing a int16array, could change to int32, (vale unitl a thousand, so 16 should be fine

    //Can the same node subscribe and publish?
    twist_sub = nh.subscribe("velocity_publisher", 1000, &ControllerNode::DesiredvelocityCallback, this);
    IMU_sub =  nh.subscribe("imu_data", 1000, &ControllerNode::IMUCallback, this);
    arduino_pub = nh.advertise<std_msgs::Int32MultiArray>("Arduino",1000);


  }
  
  ~ControllerNode(void){};
  void IMUCallback  (const sensor_msgs::Imu::ConstPtr& msg){

    IMUangularvelocity <<
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z;
    IMUlinearaccelaration << msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z;
  };
  void DesiredvelocityCallback(const  nav_msgs::Odometry::ConstPtr& desired_twist_velocity)
  {

    Eigen::MatrixXd MatrixCurrentvelocity(6,1);
    Eigen::MatrixXd MatrixDesiredvelocity(6,1);
    MatrixDesiredvelocity <<
      desired_twist_velocity->twist.twist.linear.x,
      desired_twist_velocity->twist.twist.linear.y,
      desired_twist_velocity->pose.pose.position.z,//position in real, not velocity
      desired_twist_velocity->twist.twist.angular.x,
      desired_twist_velocity->twist.twist.angular.y,
      desired_twist_velocity->twist.twist.angular.z;


    // Actual velocity to be got from the IMU and depth to be got from the pressure/depth sensor

    MatrixCurrentvelocity <<
      1,
      2,
      3,//position, not velocity
      4,
      5,
      6;

    // u = −K(x − xe) − Kiz + ud
    // u place holder
    Eigen::MatrixXd K(4,6);

    K <<
      0.3336,   -0.1862,   -0.4019,   -0.1243,    0.0020,    0.1623,
      0.2801,    0.2301,    0.4458,    0.1266,   -0.0017,   -0.1713,
      -0.0089,    0.1006,   14.5461,    0.1070,   -0.0016,    0.3206,
      0.0102,   -0.1144,   13.9227,    0.8343,    0.0004,   2-0.3244;
 
    Eigen::MatrixXd PWMmatrix(4,1);

    // PWM matrix needs to be multiplied to get the PWM value, since its in terms of torque rightnow

      //pwm = ((-k(IMU velocities)-k(intergral(y-x_desired)))*(thrust_to_pwm_const)
    PWMmatrix = K*(MatrixDesiredvelocity- MatrixCurrentvelocity); // - ki(velocity;accelaration matrix)

    // set this to the frequency of the controller
    ros::Rate loop_rate(10);



    std_msgs::Int32MultiArray motor_parameters;
    motor_parameters.data.clear();
    int T100port = 100; //temp
    int T100starboard = 100; //temp 
    int T200left = 200; //temp
    int T200right = 200;
    motor_parameters.data.push_back(PWMmatrix(1,1));
    motor_parameters.data.push_back(PWMmatrix(2,1));
    motor_parameters.data.push_back(PWMmatrix(3,1));
    motor_parameters.data.push_back(PWMmatrix(4,1));
    arduino_pub.publish(motor_parameters);


  
  }

};
int main(int argc, char **argv){

  //creates the node name something to do with command line
  ros::init(argc, argv, "motor_controller_node");
  ControllerNode Controller_Node (ros::this_node::getName());


  ros::spin();    
  return 0;

}


