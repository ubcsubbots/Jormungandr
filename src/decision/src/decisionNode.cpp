//
// Created by da-cam on 19/05/18.
//

#include "../include/decisionNode.h"


decisionNode::decisionNode(int argc, char** argv , std::string nodeName) {
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;

    subscribeTopic = "/vision/output";
    publishTopic = "/g500/twist";



    geometry_msgs::Twist msg;

    subscriber_ = nh.subscribe("/vision/output", 1, &decisionNode::subscriberCallBack, this);

    publisher_     = nh.advertise<geometry_msgs::Twist>(publishTopic, 1000);

}



void decisionNode::subscriberCallBack(const gate_detect::gateDetectMsg gateDetectMsg){

    geometry_msgs::Twist twt;

    twt.linear.x = twt.linear.y = twt.linear.z = 0;
    twt.angular.x = twt.angular.y = twt.angular.z = 0;

    if (gateDetectMsg.angleTop < 0){
        twt.linear.z = 1.0;
    }
    else if (gateDetectMsg.angleTop > 0) {
        twt.linear.z = -1.0;
    }

    publisher_.publish(twt);


}

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "my_node";

    // Create an instance of your class
    decisionNode decisionNode1
            (argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}




