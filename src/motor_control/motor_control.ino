#include <ros.h>
#include <geometry_msgs/Twist.h>
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
 * Receives Twist messages from the Decision Node
 * The robot has five degrees of freedom: 
 * Linear x, y, z, and yaw
 * 
 * The robot can move in different combinations of 
 * movement. The straight motors allow it to exclusively 
 * move in either yaw or linear x. The angled motors 
 * allow it to exclusively move in either linear y or z.
 */
void motorControlCallback (const geometry_msgs::Twist& msg){

  /********** Linear X and Yaw **********/
  //The two motions should be mutually exclusive
  // Stop the straight motors 
  if (msg.linear.x == 0 && msg.angular.z == 0){
    TLS.write(PWM_STOP);
    TRS.write(PWM_STOP);
  }
  //Linear X motion
  else if (msg.linear.x != 0 && msg.angular.z == 0){
    /* Move Forward */
    if (msg.linear.x > 0){
      TLS.write(PWM_STOP + PWM_STATIC_MOVE);
      TRS.write(PWM_STOP + PWM_STATIC_MOVE);
    }
    /* Move Backwards*/
    else{
      TLS.write(PWM_STOP - PWM_STATIC_MOVE);
      TRS.write(PWM_STOP - PWM_STATIC_MOVE);
    }  
  }
  //Angular Z motion
  else if (msg.angular.z != 0 && msg.linear.x == 0){
    /* Turn Left */
    if (msg.angular.z > 0){
      TLS.write(PWM_STOP - PWM_STATIC_MOVE);
      TRS.write(PWM_STOP + PWM_STATIC_MOVE);
    }
    /* Turn Right */
    else{
      TLS.write(PWM_STOP + PWM_STATIC_MOVE);
      TRS.write(PWM_STOP - PWM_STATIC_MOVE);
    }  
  }
  //Figure out how to handle if both are on
  else{}

  /********** Linear Y and Z **********/
  //Stop the Angled Motors
  if (!msg.linear.y && !msg.linear.z && !msg.angular.x){
    TLA.write(PWM_STOP);
    TRA.write(PWM_STOP);
  }
  //Linear Y Motion 
  else if (msg.linear.y && !msg.linear.z && !msg.angular.x){
    /* Strafe Left */
    if (msg.linear.y > 0){
      TLA.write(PWM_STOP + PWM_STATIC_MOVE);
      TRA.write(PWM_STOP - PWM_STATIC_MOVE);
    }
    /* Strafe Right */
    else{
      TLA.write(PWM_STOP - PWM_STATIC_MOVE);
      TRA.write(PWM_STOP + PWM_STATIC_MOVE);
    }  
  }
  else if (msg.linear.z && !msg.linear.y && !msg.angular.x){
    /* Climb */
    if (msg.linear.z > 0){
      TLS.write(PWM_STOP + PWM_STATIC_MOVE);
      TRS.write(PWM_STOP + PWM_STATIC_MOVE);
    }
    /* Descend*/
    else{
      TLS.write(PWM_STOP - PWM_STATIC_MOVE);
      TRS.write(PWM_STOP - PWM_STATIC_MOVE);
    }  
  }
  else {}
}

ros::Subscriber<geometry_msgs::Twist> decision_node("sub_control", motorControlCallback);

/***********************/
/*       Arduino       */
/***********************/
void setup()  {
  TLS.attach(THRUSTER_LEFT_STR);
  TLA.attach(THRUSTER_LEFT_ANG);
  TRS.attach(THRUSTER_RIGHT_STR);
  TRA.attach(THRUSTER_RIGHT_ANG);

  nh.initNode();
  nh.subscribe(decision_node);
}

void loop()  {
  nh.spinOnce();
  delay(1);
}


