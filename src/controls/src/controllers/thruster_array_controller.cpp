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

        // Initialize simulink controller
        control_system_Obj.initialize();

        startTraj = 1;
        startTime = ros::Time::now();
    
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

        // We want the trajectory to start from time 0
        if (startTraj)
        {
            startTraj = 0;
            startTime = time;
        }

        ROS_INFO("Time along trajectory %f", (ros::Time::now()-startTime).toSec());

        // Set inputs for simulink control system
        ExtU_control_system_T inputs;
        inputs.time = (real_T) (time - startTime).toSec();

        // Desired endpoint [x,y,z,phi,theta,psi]
        inputs.endpoint[0] = 0;
        inputs.endpoint[1] = 0;
        inputs.endpoint[2] = 1;
        inputs.endpoint[3] = 0;
        inputs.endpoint[4] = 0;
        inputs.endpoint[5] = 0;

        // State vector [u,v,w,r,p,y,x,y,z,phi,theta,psi]
        inputs.x[0] = 0;
        inputs.x[1] = 0;
        inputs.x[2] = 0;
        inputs.x[3] = 0;
        inputs.x[4] = 0;
        inputs.x[5] = 0;
        inputs.x[6] = 0;
        inputs.x[7] = 0;
        inputs.x[8] = 0;
        inputs.x[9] = 0;
        inputs.x[10] = 0;
        inputs.x[11] = 0;

        // Step through the simulink control system with new inputs
        control_system_Obj.control_system_U = inputs;
        control_system_Obj.step();

        // Extract outputs from simulink control system
        ExtY_control_system_T outputs = control_system_Obj.control_system_Y;
        real_T signals[6];
        signals[0] = outputs.signals[0];
        signals[1] = outputs.signals[1];
        signals[2] = outputs.signals[2];
        signals[3] = outputs.signals[3];
        signals[4] = outputs.signals[4];
        signals[5] = outputs.signals[5];

        // Fill thruster array data with signals
        ThrusterArrayData thruster_array_cmd;
        thruster_array_cmd.thruster_one_command = (float) signals[0];
        thruster_array_cmd.thruster_two_command = (float) signals[1];
        thruster_array_cmd.thruster_three_command = (float) signals[2];
        thruster_array_cmd.thruster_four_command = (float) signals[3];
        thruster_array_cmd.thruster_five_command = (float) signals[4];
        thruster_array_cmd.thruster_six_command = (float) signals[5];

        // Command the thruster array through the hardware interface
        thruster_array_handle_.commandThrusterArray(thruster_array_cmd);
        
        // ROS_INFO("Thruster Array Controller Updated");
    }

    void ThrusterArrayController::starting(const ros::Time& time) 
    {
        decision_cmd_struct_ = {0};
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


