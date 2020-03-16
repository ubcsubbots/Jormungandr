/*
 * Created By: Logan Fillo
 * Created On: March 15 2020
 * Description: A node that performs dead reckoning localization using
 *              an IMU and depth sensor
 */

#ifndef DEAD_RECKONING_NODE_H
#define DEAD_RECKONING_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <drivers_msgs/ArduinoDrivers.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <AUVState.h>
#include <queue>


class DeadReckoningNode {
  
  public:
    DeadReckoningNode(int argc, char** argv);

  private:
    /**
     * Callback function for when an Arduino drivers message is recieved
     *
     * @param msg the Arduino drivers message received in the callback
     */
    void arduinoDriversCallback(const drivers_msgs::ArduinoDrivers::ConstPtr& msg);

    /**
     * Callback function for when an IMU message is recieved
     *
     * @param msg the IMU message received in the callback
     */
    void imuDriverCallback(const sensor_msgs::Imu::ConstPtr& msg);

    ros::NodeHandle nh;

    // Driver Subscribers
    int queue_size = 10;
    ros::Subscriber arduino_drivers_sub;
    ros::Subscriber imu_driver_sub;

    // Number of samples in numeric integration 
    int n;

    // Past states to aid in numeric integration 
    std::queue<AUVState> past_states;

};
#endif // DEAD_RECKONING_NODE_H