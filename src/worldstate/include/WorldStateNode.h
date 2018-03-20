/*
 * Created By: Joel Ahn
 * Created On: March 5h, 2018
 * Description: Checks the incoming data from detection nodes
 *              and tracks the individual goals of the robot
 */

#ifndef JORMUNGANDR_WORLDSTATENODE_H
#define JORMUNGANDR_WORLDSTATENODE_H

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <worldstate/states.h>
#include <std_msgs/builtin_int8.h>

//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <vision/TutorialsConfig.h>

class WorldStateNode {

public:
    WorldStateNode(int argc, char** argv, std::string node_name);

private:
    /**
     * TODO: Change the parameter to receive gate detection topics
     * Callback function for when data is received from gate detection node
     *
     * @param gate detection node discretized messages
     */
    void gateDetectCallBack(const std_msgs::Int8ConstPtr &num);

    /**
     *
     */
    void publishWorldState(int8_t num);

    enum internalWorldStates {
        locatingGate,
        aligningWithGate,
        passingGate,
        locatingPole,
        approachingPole,
        pivotingPole
    };

    /* Temporarily directly receive hsv-filtered msgs */
    //image_transport::Subscriber gate_node_subscriber_;
    ros::Subscriber test_;
    ros::Publisher world_state_publisher_;

    /* Robot should always start off by searching for the starting gate*/
    internalWorldStates current_state_ = locatingGate;

};

#endif //JORMUNGANDR_WORLDSTATENODE_H
