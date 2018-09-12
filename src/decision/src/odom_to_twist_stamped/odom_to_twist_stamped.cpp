/*
 * Created By: Cameron Newton
 * Created On: Sept 12nd, 2018
 * Description: A node that remaps the output of the decision node from an Odom
 * message
 *              to a TwistStamped for the simulator
 */

#include <OdomToTwistStamped.h>

int main(int argc, char** argv) {
    std::string node_name = "odom_to_twist_stamped";

    OdomToTwistStamped odomToTwistStamped(argc, argv, node_name);

    ros::spin();

    return 0;
}
