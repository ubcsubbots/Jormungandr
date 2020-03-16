/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Thruster array controller header
 */

#ifndef THRUSTER_CONTROLLERS__THRUSTER_ARRAY_CONTROLLER_H
#define THRUSTER_CONTROLLERS__THRUSTER_ARRAY_CONTROLLER_H

#include <realtime_tools/realtime_buffer.h>
#include <AUVState.h>
#include<ThrusterArrayCommand.h>
#include <controller_interface/controller.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <pluginlib/class_list_macros.hpp>
#include <interface/thruster_array_interface.h>
#include <simulink/control_system.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

namespace thruster_controllers
{
    /**
    * @class thruster_controllers::ThrusterArrayController
    * 
    * Controls output of a thruster array using a control system
    * 
    * @param type Must be "thruster_controllers/ThrusterArrayController"
    * @param topics A map of topics which this controller needs 
    * 
    * Subscribes to:
    * 
    * - topics.decision (nav_msgs::Odometry): the navigation decision command
    * 
    */
    class ThrusterArrayController: public controller_interface::Controller<hardware_interface::ThrusterArrayInterface>
    {
    public:
        
        ThrusterArrayController();
        ~ThrusterArrayController();

        /**
         * Initializes the thrusters controller, makes sure the correct thruster configuration
         * is found in the thrusters command interface. Also sets up subscribers.
         * 
         * @param robot Pointer to the thruster command interface to be used by this controller
         * @param root_nh Nodehandle in the root namespace
         * @param controller_nh Nodehandle in the namespace of where the controller should be configured
         */
        bool init(hardware_interface::ThrusterArrayInterface *robot, 
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

        /**
         * Called by the controller manager when it is time to update the controllers from the
         * realtime control loop. 
         * 
         * Calculates the command to be sent to each thruster using the simulink control system
         * 
         * @param time The time when this update execution is called
         * @param period The amount of time since the update was last executed
         */
        void update(const ros::Time& time, const ros::Duration& period); 

        /**
         * Called when controller is starting in realtime loop
         * 
         * Initializes the realtime buffers for subscriber callbacks 
         * and resets the internal PID controller
         * 
         * @param time The current time
         */
        void starting(const ros::Time& time);

        /**
         * Called when controller is stopping in realtime loop
         * 
         * Shuts down this controller's subscribers
         * 
         * @param time The current time
         */
        void stopping(const ros::Time& time);

    private:

        // Subscriber callback
        void decisionCB(const nav_msgs::Odometry::ConstPtr& msg);
        ros::Subscriber decision_sub_;
        static const int msg_queue_ = 10; 

        // Hardware interface components
        hardware_interface::ThrusterArrayHandle thruster_array_handle_;

        // Realtime buffer to recieve messages in realtime
        realtime_tools::RealtimeBuffer<AUVState> desired_state_;

        // Memory allocated for subscriber messages buffer
        AUVState desired_state_struct_;

        // Generated simulink controller object
        control_system2ModelClass control_system_Obj;

        // Trajectory timing
        bool newTrajectory;
        ros::Time trajectoryStartTime ;
          
    };
    
} // namespace thruster_controllers

#endif // THRUSTER_CONTROLLERS__THRUSTER_ARRAY_CONTROLLER_H