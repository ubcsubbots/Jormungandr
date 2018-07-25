#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Empty.h>
#include <Servo.h>

/***********************/
/*        Servo        */
/***********************/

// Define the motor ESC pins
#define THRUSTER_LEFT_STR  3
#define THRUSTER_LEFT_ANG  5
#define THRUSTER_RIGHT_STR 6
#define THRUSTER_RIGHT_ANG 9

// Define some ESC PWM calibration constants
#define PWM_STOP        91
#define PWM_STATIC_MOVE 50

Servo TLS; //Thruster on Left pointing Straight
Servo TLA; //Thruster on Left Angled 45
Servo TRS; //Thruster on Right pointing Straight
Servo TRA; //Thruster on Right Angled 45

/***********************/
/*       Safety        */
/***********************/
// Maximum time we will allow between the computer seeing messages
// from us before we think the e-stop is triggered
#define MAX_TIME_SINCE_LAST_ALIVE_SECONDS 0.5

// Time between sending out "is_alive" messages to the computer
#define TIME_BETWEEN_IS_ALIVE_MSGS_MILLIS 400

// Whether or not we think the e-stop was triggered
bool e_stop_triggered;  

// Whether or not we've received at least one message from the computer
// indicating how long it's been since it received a "is_alive" msg
// from us
bool first_is_alive_timer_recieved;

// How long it's been since we told the system we were alive
unsigned long last_is_alive_msg_sent_millis;

/**
 * Sets all motor speeds to 0
 */
void stopAllMotors(){
  TRS.writeMicroseconds(0);
  TLS.writeMicroseconds(0);
  TRA.writeMicroseconds(0);
  TLA.writeMicroseconds(0);
}

/***********************/
/*         ROS         */
/***********************/
ros::NodeHandle nh;

/**
 * Receives a message from the computer how long it's been 
 * since we were last seen alive. If it's been too long, 
 * then an e-stop was probably triggered and we should
 * stop moving
 */
void isAliveTimeCallback(const std_msgs::Duration& msg){
    first_is_alive_timer_recieved = true;

    // If e-stop was already triggered, don't change anything
    if (e_stop_triggered){
        return;
    }

    float time_since_last_alive = msg.data.toSec();
    if (time_since_last_alive > MAX_TIME_SINCE_LAST_ALIVE_SECONDS){
        e_stop_triggered = true;
        stopAllMotors();
    }
}

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
  if (!e_stop_triggered){
    TRS.writeMicroseconds(msg.data[0]);
    TLS.writeMicroseconds(msg.data[1]);
    TRA.writeMicroseconds(msg.data[2]);
    TLA.writeMicroseconds(msg.data[3]);
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> motor_arduino_node("Arduino", motorControlCallback);
ros::Subscriber<std_msgs::Duration> is_alive_duration_subscriber(
    "/dead_mans_switch/time_since_last_is_alive", 
    isAliveTimeCallback
);

std_msgs::Empty is_alive_msg;
ros::Publisher is_alive_publisher("/dead_mans_switch/is_alive_input", &is_alive_msg);

/***********************/
/*       Arduino       */
/***********************/
void setup()  {
  TLS.attach(THRUSTER_LEFT_STR);
  TLA.attach(THRUSTER_LEFT_ANG);
  TRS.attach(THRUSTER_RIGHT_STR);
  TRA.attach(THRUSTER_RIGHT_ANG);

  // Motors should be off to start
  stopAllMotors();

  nh.initNode();
  nh.subscribe(is_alive_duration_subscriber);

  // Let the rest of the system know we're alive
  std_msgs::Empty is_alive;
  is_alive_publisher.publish(&is_alive);

  // Wait until we get at least one message from the computer indicating how
  // long it's been since we were last seen alive, we might be turning on again
  // after the e-stop was triggered
  while (!first_is_alive_timer_recieved){
    nh.spinOnce();
    delay(1);
  }

  // Once we've figured out if the e-stop has been triggered or not 
  // (at least initially) we can start listening for motor commands
  // and letting the system know we're alive
  nh.subscribe(motor_arduino_node);
  nh.advertise(is_alive_publisher);
}

void loop()  {
  // Do nothing if the e-stop has been triggered
  if (e_stop_triggered){
    
    // If it's been long enough, let the computer know we're still alive
    unsigned long curr_time_millis = millis();
    if (curr_time_millis - last_is_alive_msg_sent_millis > TIME_BETWEEN_IS_ALIVE_MSGS_MILLIS){
      std_msgs::Empty is_alive;
      is_alive_publisher.publish(&is_alive);
      last_is_alive_msg_sent_millis = curr_time_millis;
    } 

    nh.spinOnce();
  }
  delay(1);
}


