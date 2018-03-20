/*
 * Created By: Joel Ahn
 * Created On: March 5h, 2018
 * Description: Checks the incoming data from detection nodes
 *              and tracks the individual goals of the robot
 */

#include <WorldStateNode.h>

WorldStateNode::WorldStateNode(int argc, char** argv, std::string node_name){
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    //image_transport::ImageTransport it(nh);

    //Change the subscribe topics as needed
    std::string gateDetectionSubscription = "/vision/output";
    std::string publishTopic   = "worldstate/output";

    world_state_publisher_ = nh.advertise<worldstate::states>(publishTopic, 100);
    test_      = nh.subscribe(
            "test/temp", 1, &WorldStateNode::gateDetectCallBack, this);

    ros::spin();
}

/**
     * TODO: What is the topic being received from the gate detection node?
     * Callback function for when data is received from gate detection node
     *
     * @param image the image received in the callback
     */
void WorldStateNode::gateDetectCallBack(const std_msgs::Int8ConstPtr &num){
    publishWorldState(num->data);
}

void WorldStateNode::publishWorldState(int num){

    switch (num){
        /*case 0:
            world_state_publisher_.publish(worldstate::states::locatingGate);
            break;
        case 1:
            world_state_publisher_.publish(worldstate::states::aligningWithGate);
            break;
        case 2:
            world_state_publisher_.publish(worldstate::states::passingGate);
            break;
        case 3:
            world_state_publisher_.publish(worldstate::states::locatingPole);
            break;
        case 4:
            world_state_publisher_.publish(worldstate::states::approachingPole);
            break;
        case 5:
            world_state_publisher_.publish(worldstate::states::pivotingPole);
            break;*/
        default:
            break;

    }

    world_state_publisher_.publish();
}

