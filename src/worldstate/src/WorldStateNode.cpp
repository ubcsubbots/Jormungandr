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

    world_state_publisher_ = nh.advertise<worldstate::state_msg>(publishTopic, 100);
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
    worldstate::state_msg msg;
    worldstate::state_msg_<int8_t> test;

    switch (num->data){
        case 0:
            msg.state = test.locatingPole; //worldstate::state_msg_::locatingGate;
            break;
        case 1:
            msg.state = test.aligningWithGate; //worldstate::state_msg_::aligningWithGate;
            break;
        case 2:
            msg.state = test.passingGate; //worldstate::state_msg_::passingGate;
            break;
        case 3:
            msg.state = test.locatingPole; //worldstate::state_msg_::locatingPole;
            break;
        case 4:
            msg.state = test.approachingPole; //worldstate::state_msg_::approachingPole;
            break;
        case 5:
            msg.state = test.pivotingPole; //worldstate::state_msg_::pivotingPole;
            break;
        default:
            msg.state = test.locatingGate; //worldstate::state_msg_::locatingGate;
            break;
    }

    world_state_publisher_.publish(msg);
}


