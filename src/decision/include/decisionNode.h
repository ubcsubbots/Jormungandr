//
// Created by da-cam on 19/05/18.
//

#ifndef PROJECT_DECISIONNODE_H
#define PROJECT_DECISIONNODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gate_detect/gateDetectMsg.h>

class decisionNode {
public:
    std::string subscribeTopic, publishTopic;

    std::vector<float> floater;

    decisionNode(int argc, char** argv , std::string nodeName);

private:

    void subscriberCallBack(const gate_detect::gateDetectMsg gateDetectMsg);

    void publisherCallback(const geometry_msgs::Twist twist);

    ros::Publisher publisher_;

    ros::Subscriber subscriber_;

};


#endif //PROJECT_DECISIONNODE_H
