/*
 * Created By: Gareth Ellis
 * Created On: July 22nd, 2018
 * Description: A node that publishes how long it's been since it received a
 *              message on a given topic. This can be used by other nodes
 *              to check if they have died and how long ago. Currently it's
 *              used by the firmware running on the Arduino to check if the
 *              safety has previously been triggered (and so cut power to
 *              the Arduino).
 */

#include <DeadMansSwitchNode.h>

int main(int argc, char** argv) {
    std::string node_name = "dead_mans_switch";

    DeadMansSwitchNode dead_mans_switch_node(argc, argv, node_name);

    ros::spin();

    return 0;
}