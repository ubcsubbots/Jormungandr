#ifndef THRUSTER_CONTROLLERS__MULTI_THRUSTER_CONTROLLER_H
#define THRUSTER_CONTROLLERS__MULTI_THRUSTER_CONTROLLER_H

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <interface/thruster_command_interface.h>

namespace thruster_controllers
{
    /**
    * @class thruster_controllers::MultiThrusterController
    * 
    * Controls output of multiple thrusters using a pid loop.
    * 
    * @param type Must be "thruster_controllers/MultiThrusterController"
    * @param num_thrusters Number of thrusters to control
    * @param names A list of the names of the thrusters (must be length of num_thrusters)
    * @param topics A map of topics which this controller needs 
    * @param pid The gains for the pid cotnroller
    * 
    * Subscribes to:
    * 
    * - @b topics.decision (nav_msgs::Odometry): the navigation state to achieve
    * 
    * - @b topics.imu (sensor_msgs::Imu): the imu state
    * 
    * - @b topics.depth (std_msgs::Float64): the depth state
    */
    class MultiThrusterController: public controller_interface::Controller<hardware_interface::ThrusterCommandInterface>
    {
    public:
        
        /**
         * Struct to store Imu data for use by the realtime buffer 
         */
        struct ImuData
        {
            /* Imu data params*/
        };

        /**
         * Struct to store Decision command for use by the realtime buffer 
         */
        struct DecisionCmd
        {
            /* Decision command params */
        };
        
        MultiThrusterController();
        ~MultiThrusterController();

        /**
         * Initializes the thrusters controller, makes sure the correct thruster configuration
         * is found in the thrusters command interface. Also sets up subscribers to decision 
         * output, imu sensor, and depth sensor.
         * 
         * @param robot Pointer to the thruster command interface to be used by this controller
         * @param root_nh Nodehandle in the root namespace
         * @param controller_nh Nodehandle in the namespace of where the controller should be configured
         */
        bool init(hardware_interface::ThrusterCommandInterface *robot, 
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

        /**
         * Called by the controller manager when it is time to update the controllers from the
         * realtime control loop. 
         * 
         * Calculates the command to be sent to each thruster using the current and previous 
         * decision command, imu state, and depth state. Uses the internal pid controller to 
         * determine the actual command sent to the thrusters with the calcuated error. Once
         * each command is finalized, uses each thruster's handle to send the command to the
         * hardware interface.
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

        // Subscriber callbacks
        void decisionCB(const nav_msgs::Odometry::ConstPtr& msg);
        void imuCB(const sensor_msgs::Imu::ConstPtr& msg);
        void depthCB(const std_msgs::Float64::ConstPtr& msg);

        // Hardware interface components
        int num_thrusters_;
        std::vector<std::string> thruster_names_;
        std::map<std::string, hardware_interface::ThrusterHandle> thruster_handles_;

        // Internal PID controller
        control_toolbox::Pid pid_controller_; 

        // Subscribers
        ros::Subscriber decision_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber depth_sub_;
        static const int msg_queue_ = 10; 

        // Realtime buffer to recieve callbacks
        realtime_tools::RealtimeBuffer<DecisionCmd> decision_cmd_;
        realtime_tools::RealtimeBuffer<ImuData> imu_data_ ;
        realtime_tools::RealtimeBuffer<float>  depth_state_;

        // Memory allocated for buffer
        DecisionCmd decision_cmd_struct_;
        ImuData imu_data_struct_;
        float depth_state_val_;
          
    };
    
} // namespace thruster_controllers

#endif // THRUSTER_CONTROLLERS__MULTI_THRUSTER_CONTROLLER_H