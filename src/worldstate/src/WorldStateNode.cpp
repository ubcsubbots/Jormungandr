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
    gate_detect_listener_  = nh.subscribe(gateDetectionSubscription, 1, &WorldStateNode::gateDetectCallBack, this);

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
    /*TODO Update code when gate detect node custom msg is available*/
    /*
     * Implement (for the most part) a memoryless finite state machine
     * Left -> k = 1;  Right -> k = 2;  Top -> k = 3
     * int   G   = number of gate bars seen
     * float D_k = distance from camera to bar k
     * bool  g_k = bar k visible
     */

    /* State! G = 0
     * output is "locatingGate" unless previous state was passingGate,
     * in which case the output is "passingGate." This is under the assumption
     * that the robot is moving through the gate and the bars have simply
     * disappeared from view
     */

    /* State! G = 1
     * output is "aligningWithGate" A pole has been found and the robot should
     * take the appropriate maneuvers to align with the gate appropriately.
     * Possible exception case is if the previous state was "passingGate" and the
     * top bar still remains in view
     *
     */

    /* State! G = 2
     * - If (g_1 && g_2) but (D_1 != D_2 within some error threshold), then "aligningWithGate"
     * - If g_1 and g_2 and D_1 == D_2 within some error threshold, then "passingGate"
     * - If !(g_1 && g_2), then "aligningWithGate"
     */

    /* State! G = 3
     * -If (D_1 != D_2), then "aligningWithGate"
     * -This one might be more trouble than it's worth but: if (D_3 > clearanceHeight) then "aligningWithGate"
     * -If (D_1 == D_2) and possibly if D_3 <= clearanceHeight, then "passingGate"
     *
     */
    
    switch (num->data){
        case 0:
            msg.state = worldstate::state_msg_<u_int8_t>::locatingGate;
            break;
        case 1:
            msg.state = worldstate::state_msg_<u_int8_t>::aligningWithGate;
            break;
        case 2:
            msg.state = worldstate::state_msg_<u_int8_t>::passingGate;
            break;
        case 3:
            msg.state = worldstate::state_msg_<u_int8_t>::locatingPole;
            break;
        case 4:
            msg.state = worldstate::state_msg_<u_int8_t>::approachingPole;
            break;
        case 5:
            msg.state = worldstate::state_msg_<u_int8_t>::pivotingPole;
            break;
        default:
            msg.state = worldstate::state_msg_<u_int8_t>::locatingGate;
            break;
    }

    world_state_publisher_.publish(msg);
}


