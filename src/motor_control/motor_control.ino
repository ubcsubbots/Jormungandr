#include <ros.h>
#include <std_msgs/Twist.h>
#include <Servo.h>

/***********************/
/*        Servo        */
/***********************/

//Define the motor ESC pins
#define THRUSTER_LEFT_STR  3
#define THRUSTER_LEFT_ANG  5
#define THRUSTER_RIGHT_STR 6
#define THRUSTER_RIGHT_ANG 9

//Define some ESC PWM calibration constants
#define PWM_STOP        91
#define PWM_STATIC_MOVE 50
Servo TLS; //Thruster on Left pointing Straight
Servo TLA; //Thruster on Left Angled 45
Servo TRS; //Thruster on Right pointing Straight
Servo TRA; //Thruster on Right Angled 45

/***********************/
/*         ROS         */
/***********************/
ros::NodeHandle nh;

/**
 * Receives Int16 Multi Array messages from the Controller Node
 * The robot has five degrees of freedom: 
 * Linear x, y, z, and yaw
 * 
 * The robot can move in different combinations of 
 * movement. The straight motors allow it to exclusively 
 * move in either yaw or linear x. The angled motors 
 * allow it to exclusively move in either linear y or z.
 */
void motorControlCallback (const std_msgs::Int16MultiArray& msg){
  TRS.writeMicroseconds(msg.data[0]);
  TLS.writeMicroseconds(msg.data[1]);
  TRA.writeMicroseconds(msg.data[2]);
  TLA.writeMicroseconds(msg.data[3]);
}

ros::Subscriber<std_msgs::Int16MultiArray> motor_arduino_node("Arduino", motorControlCallback);

/***********************/
/*       Arduino       */
/***********************/
void setup()  {
  TLS.attach(THRUSTER_LEFT_STR);
  TLA.attach(THRUSTER_LEFT_ANG);
  TRS.attach(THRUSTER_RIGHT_STR);
  TRA.attach(THRUSTER_RIGHT_ANG);

  nh.initNode();
  nh.subscribe(motor_arduino_node);
}

void loop()  {
  nh.spinOnce();
  delay(1);
}


