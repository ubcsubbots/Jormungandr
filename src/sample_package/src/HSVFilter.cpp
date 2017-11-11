/*
 * Created By: Reid Oliveira
 * Created On: November 11th, 2017
 * Description:
 */

#include <HSVFilter.h>

HSVFilter::HSVFilter(int argc, char** argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
}

void HSVFilter::subscriberCallBack(const sensor_msgs::Image image) {

}