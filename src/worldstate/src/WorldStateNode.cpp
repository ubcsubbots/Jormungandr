/*
 * Created By: Joel Ahn
 * Created On: March 5h, 2018
 * Description: Checks the incoming data from detection nodes
 *              and tracks the individual goals of the robot
 */

#include <WorldStateNode.h>
#include <math.h>

WorldStateNode::WorldStateNode(int argc, char** argv, std::string node_name){
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    //Change the subscribe topics as needed
    std::string gateDetectionSubscription = "/test/temp";
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
void WorldStateNode::gateDetectCallBack(const gate_detect::gateDetectMsgConstPtr & gateDetMsg) {
    worldstate::state_msg msg;
    double distBtwnHorizontalGatePole = fabs(gateDetMsg->distanceLeft - gateDetMsg->distanceRight);
    float errorTolerance = 0.25; // Assume an error tolerance of 25 cm
    float clearanceHeight = 0.1; // Assume that sub requires 10 cm of clearance to pass

    switch (current_state_) {
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

        case locatingGate:
            //If any of the poles has been found
            if (gateDetMsg->detectLeft || gateDetMsg->detectRight || gateDetMsg->detectTop)
                current_state_ = aligningWithGate;
            //If all three poles are in view
            if (gateDetMsg->detectLeft && gateDetMsg->detectRight && gateDetMsg->detectTop){
                // and the robot is relatively aligned
                if (distBtwnHorizontalGatePole < errorTolerance /*&& gateDetMsg->distanceTop < clearanceHeight*/)
                    current_state_ = passingGate;
            }
            break;

        case aligningWithGate:
            //If the poles suddenly disappear from view
            if (!gateDetMsg->detectTop && !gateDetMsg->detectLeft && !gateDetMsg->detectRight){
                current_state_ = locatingGate;
            }

            //Assuming all poles are in view
            if (gateDetMsg->detectTop && gateDetMsg->detectLeft && gateDetMsg->detectRight){
                // and the robot is relatively aligned
                if (distBtwnHorizontalGatePole < errorTolerance /*&& gateDetMsg->distanceTop < clearanceHeight*/)
                    current_state_ = passingGate;
            }
            break;

        case passingGate:
            //If the poles suddenly disappear from view, the robot should bee-line to
            //pass through the gate
            if (!gateDetMsg->detectTop && !gateDetMsg->detectLeft && !gateDetMsg->detectRight){
                current_state_ = passingGate;
            }
            break;

        default:
            break;
    }

    msg.state = current_state_;

    world_state_publisher_.publish(msg);
}


