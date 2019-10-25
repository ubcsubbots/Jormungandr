/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Depth state controller header
 */

#ifndef SENSOR_CONTROLLERS__DEPTH_STATE_CONTROLLER_H
#define SENSOR_CONTROLLERS__DEPTH_STATE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_publisher.h>
#include <interface/depth_state_interface.h>
#include <boost/shared_ptr.hpp>

namespace sensor_controllers
{
    /**
    * @class sensor_controllers::DepthStateController
    * 
    * Reads and publishes depth state in control loop
    * 
    * @param type Must be "sensor_controllers/DepthStateController"
    * @param publish_rate The rate at which messages are published
    * 
    * Publish to
    * 
    * - @b /${name of the handle defined in hardware interface} (std_msgs::Float64)
    */
    class DepthStateController : public controller_interface::Controller<hardware_interface::DepthStateInterface>
    {
    public:

        DepthStateController();
        ~DepthStateController();

        /**
         * Initializes the depth state controller
         * 
         * @param robot Pointer to the depth state interface to be used by this controller
         * @param root_nh Nodehandle in the root namespace
         * @param controller_nh Nodehandle in the namespace of where the controller should be configured
         */
        bool init(hardware_interface::DepthStateInterface *robot, 
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

        /**
         * Called by the controller manager when it is time to update the controllers from the
         * realtime control loop. 
         * 
         * Publishes depth state data in realtime if this update execution corresponds to the publish rate
         * 
         * @param time The time when this update execution is called
         * @param period The amount of time since the update was last executed
         */
        void update(const ros::Time& time, const ros::Duration& period); 

        /**
         * Called when controller is starting in realtime loop
         * 
         * @param time The current time
         */
        void starting(const ros::Time& time);

        /**
         * Called when controller is stopping in realtime loop
         * 
         * @param time The current time
         */
        void stopping(const ros::Time& time);


    private:

        // Hardware interface component
        hardware_interface::DepthStateHandle depth_sensor_;

        // Realtime publisher
        typedef boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64> > RtPublisherPtr;
        RtPublisherPtr realtime_pub_;

        // Publish time data
        ros::Time last_publish_time_;
        double publish_rate_;
    };
    
} // namespace sensor_controllers



#endif // SENSOR_CONTROLLERS__DEPTH_STATE_CONTROLLER_H