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

        newTrajectory = 1;
        trajectoryStartTime = ros::Time::now();
    
        ROS_INFO("Thruster Array Controller Initialized");
        return true;
    }

    void ThrusterArrayController::update(const ros::Time& time, const ros::Duration& period) 
    {   
        // Get latest decision command
        desired_state_struct_ = *( desired_state_.readFromRT() );

        // Read current state data from shared interface
        AUVState auv_state = *(thruster_array_handle_.getAUVState());

        // We want the a new trajectory to start from time 0
        if (newTrajectory)
        {
            newTrajectory = 0;
            trajectoryStartTime = time;
        }

        // ROS_INFO("Time along trajectory %f", (ros::Time::now()-trajectoryStartTime).toSec());

        // Set inputs for simulink control system
        ExtU_control_system_T inputs;
        inputs.time = (real_T) (time - trajectoryStartTime).toSec();

        // Desired endpoint pose [x,y,z,phi,theta,psi]
        inputs.endpoint[0] = desired_state_struct_.x;
        inputs.endpoint[1] = desired_state_struct_.y;
        inputs.endpoint[2] = desired_state_struct_.z;
        inputs.endpoint[3] = desired_state_struct_.phi;
        inputs.endpoint[4] = desired_state_struct_.theta;
        inputs.endpoint[5] = desired_state_struct_.psi;

        // State vector [u,v,w,p,q,r,x,y,z,phi,theta,psi]
        inputs.x[0] = auv_state.u;
        inputs.x[1] = auv_state.v;
        inputs.x[2] = auv_state.w;
        inputs.x[3] = auv_state.p;
        inputs.x[4] = auv_state.q;
        inputs.x[5] = auv_state.r;
        inputs.x[6] = auv_state.x;
        inputs.x[7] = auv_state.y;
        inputs.x[8] = auv_state.z;
        inputs.x[9] = auv_state.phi;
        inputs.x[10] = auv_state.theta;
        inputs.x[11] = auv_state.psi;

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

        // Fill thruster array command with signals
        ThrusterArrayCommand thruster_array_cmd;
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
        desired_state_struct_ = {0};
        desired_state_.initRT(desired_state_struct_);
    }

    void ThrusterArrayController::stopping(const ros::Time& time) 
    {
        decision_sub_.shutdown();
    }

    void ThrusterArrayController::decisionCB(const nav_msgs::Odometry::ConstPtr& msg) 
    {   
        // ROS_INFO("Got decision message");
        AUVState decision_state;
        decision_state.x = msg->pose.pose.position.x;
        decision_state.y = msg->pose.pose.position.y;
        decision_state.z = msg->pose.pose.position.z;
        
        // Convert quat orientation to euler angles (phi theta psi)
        tf::Quaternion quat( msg->pose.pose.orientation.x, 
                             msg->pose.pose.orientation.y, 
                             msg->pose.pose.orientation.z, 
                             msg->pose.pose.orientation.w);

        tf::Matrix3x3 mat(quat);
        double phi, theta, psi;
        mat.getRPY(phi, theta, psi);
        decision_state.phi = phi;
        decision_state.phi = theta;
        decision_state.phi = psi;
        desired_state_struct_ = decision_state;
        desired_state_.writeFromNonRT(desired_state_struct_);
    }

} // namespace thruster_controllers

PLUGINLIB_EXPORT_CLASS(thruster_controllers::ThrusterArrayController, controller_interface::ControllerBase)


