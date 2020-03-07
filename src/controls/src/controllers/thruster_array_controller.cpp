/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Thruster array controller
 */

#include <controllers/thruster_array_controller.h>

namespace thruster_controllers {

    ThrusterArrayController::ThrusterArrayController() {}

    ThrusterArrayController::~ThrusterArrayController() {}

    bool ThrusterArrayController::init(hardware_interface::ThrusterArrayInterface *robot, 
        ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {   
        thruster_array_handle_ = robot->getHandle("thruster_array");

        std::map<std::string,std::string> topics;  
        if (!controller_nh.getParam("topics", topics))
        {
            ROS_ERROR("Couldn't retrieve thruster array topics");
            return false;
        }

        decision_sub_ = root_nh.subscribe<nav_msgs::Odometry>(
            topics["decision"], msg_queue_, 
            &ThrusterArrayController::decisionCB, this);

        ROS_INFO("Thruster Array Controller Initialized");
        return true;
    }

    void ThrusterArrayController::update(const ros::Time& time, const ros::Duration& period) 
    {   
        // Get latest decision command
        decision_cmd_struct_ = *( decision_cmd_.readFromRT() );

        // Read state data from shared interface
        ImuSensorData imu_data = *(thruster_array_handle_.getImuSensor());
        DepthSensorData depth_data = *(thruster_array_handle_.getDepthSensor());

        // Call the step function of the MATLAB Simulink generated controller
        // using the latest state information, and call thruster_array_handle.commandThrusterArray(signals)
        // where signals is the output of the controller stored in ThrusterArrayData

        // ROS_INFO("Thruster Array Controller Updated");
    }

    void ThrusterArrayController::starting(const ros::Time& time) 
    {
        // TODO: Initialize decision command struct
        decision_cmd_.initRT(decision_cmd_struct_);
    }

    void ThrusterArrayController::stopping(const ros::Time& time) 
    {
        decision_sub_.shutdown();
    }

    void ThrusterArrayController::decisionCB(const nav_msgs::Odometry::ConstPtr& msg) 
    {   
        // ROS_INFO("Got decision message");
        DecisionCmd cmd;
        // TODO: fill cmd with msg
        decision_cmd_struct_ = cmd;
        decision_cmd_.writeFromNonRT(decision_cmd_struct_);
    }

} // namespace thruster_controllers

PLUGINLIB_EXPORT_CLASS(thruster_controllers::ThrusterArrayController, controller_interface::ControllerBase)


