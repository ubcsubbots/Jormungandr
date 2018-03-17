/*
 * Created By: reidoliveira
 * Created On: March 17, 2018
 * Description:
 */
#ifndef DECISION_LOCATEGATE_H
#define DECISION_LOCATEGATE_H

#include <std_msgs/String.h>
#include "Subroutine.h"

class LocateGate: public Subroutine {
public:
    LocateGate(int argc, char **argv, std::string node_name): Subroutine(argc, argv, node_name) {}
    void startup() override;
    void shutdown() override;
    void publishCommand(const geometry_msgs::Twist::ConstPtr &msg) override;
private:
    void setupSubscriptions(ros::NodeHandle nh) override;

    void firstSubscriberCallback(const std_msgs::String::ConstPtr& msg);

};

#endif //DECISION_LOCATEGATE_H
