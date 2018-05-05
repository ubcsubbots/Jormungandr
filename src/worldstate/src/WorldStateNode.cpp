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

    //Change the subscribe topics as needed
    std::string gateDetectionSubscription = "/gateDetect/output";
    std::string publishTopic   = "worldstate/output";

    world_state_publisher_ = nh.advertise<worldstate::state_msg>(publishTopic, 100);
    gate_detect_listener_  = nh.subscribe(gateDetectionSubscription, 1, &WorldStateNode::gateDetectCallBack, this);

    ros::spin();
}

/**
 * Callback function for when data is received from gate detection node
 *
 * @param image the image received in the callback
 */
void WorldStateNode::gateDetectCallBack(const gate_detect::gateDetectMsgConstPtr & msg) {
    worldstate::state_msg gateDetMsg;
    double distBtwnHorizontalGatePole = fabs(msg->distanceLeft - msg->distanceRight);
    float errorTolerance = 0.25; // Assume an error tolerance of 25 cm
    float clearanceHeight = 0.1; // Assume that sub requires 10 cm of clearance to pass

    switch (current_state_) {
        /*
         * Implement (for the most part) a memoryless state machine
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
         */

        /* State! G = 2 or 3
         * - If g_1, g_2, and g_3, and |D_1 - D_2| < error and D_3 >= clearance then "passingGate"
         * - If !g_1, !g_2, and !g_3, go back to "locatingGate"
         * - Otherwise "aligningWithGate"
         */

        case locatingGate:
            //If any of the poles has been found
            if (msg->detectLeft || msg->detectRight || msg->detectTop) {
                current_state_ = aligningWithGate;
                //If all three poles are in view
                if (msg->detectLeft && msg->detectRight) {
                    // and the robot is relatively aligned
                    if (distBtwnHorizontalGatePole < errorTolerance && msg->distanceTop >= clearanceHeight)
                        current_state_ = passingGate;
                }
            }
            break;

        case aligningWithGate:
            //If the poles suddenly disappear from view
            if (!msg->detectTop && !msg->detectLeft && !msg->detectRight){
                current_state_ = locatingGate;
            }

            //Assuming all poles are in view
            if (msg->detectTop && msg->detectLeft && msg->detectRight){
                // and the robot is relatively aligned
                if (distBtwnHorizontalGatePole < errorTolerance && msg->distanceTop >= clearanceHeight)
                    current_state_ = passingGate;
            }
            break;

        case passingGate:
            /* Handle Error Exceptions */


            //If the poles suddenly disappear from view, assume the robot has bee-lined and
            //passed through the gate successfully
            if (!msg->detectTop && !msg->detectLeft && !msg->detectRight){
                current_state_ = locatingPole;
            }
            break;

        default:
            break;
    }

    gateDetMsg.state = current_state_;

    world_state_publisher_.publish(gateDetMsg);
}

void WorldStateNode::poleDetectCallBack(void) {

}

