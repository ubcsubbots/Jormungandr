/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Depth state controller
 */

#include <controllers/depth_state_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace sensor_controllers
{
    DepthStateController::DepthStateController() {}

    DepthStateController::~DepthStateController() {}

    bool DepthStateController::init(hardware_interface::DepthStateInterface *robot, 
                                    ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        if (!controller_nh.getParam("publish_rate", publish_rate_))
        {
            ROS_ERROR("Parameter 'publish_rate' not set");
            return false;
        }

        std::string name = robot->getNames()[0];
        depth_sensor_ = robot->getHandle(name);

        realtime_pub_.reset(
            new realtime_tools::RealtimePublisher<std_msgs::Float64>(root_nh, name, 4));
        realtime_pub_->msg_.data = 0.0;

        ROS_INFO("Depth State Controller Initialized");
        return true;
    }

    void DepthStateController::update(const ros::Time& time, const ros::Duration& period) 
    {   
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){
            if (realtime_pub_ && realtime_pub_->trylock()){
                last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
                realtime_pub_->msg_.data = depth_sensor_.getState();
                realtime_pub_->unlockAndPublish();
            }
        }
        // ROS_INFO("Depth State Controller Updated");
    }

    void DepthStateController::starting(const ros::Time& time) {
        last_publish_time_ = time;
    }

    void DepthStateController::stopping(const ros::Time& time) {}

} // namespace sensor_controllers

PLUGINLIB_EXPORT_CLASS(sensor_controllers::DepthStateController, controller_interface::ControllerBase)


