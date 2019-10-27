/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Control node 
 */

#include <interface/robot_hardware_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/console.h>


int main(int argc, char** argv) {

    // Setup our ROS node
    std::string node_name = "control_node";
    ros::init(argc, argv, node_name);

    // Create a controller manager for an instance of our robot hardware interface
    RobotHardwareInterface robot(argc, argv, node_name);
    controller_manager::ControllerManager cm(&robot);

    // Setup threading model so callbacks are executed on seperate thread
    // Argument of 0 to spinner means it uses num of cores the ros host has
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Define timing data
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(1.0);

    while (ros::ok())
    {
        // Update timing data
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time; // is this right??

        // Realtime control loop
        robot.read();
        cm.update(time, period);
        robot.write();

        // Sleep for the leftover time in an iteration of control loop
        ROS_INFO("Control loop executed in %f seconds ", (ros::Time::now()-time).toSec());
        rate.sleep();
    }

    // Once the node stops, return 0
    return 0;
}