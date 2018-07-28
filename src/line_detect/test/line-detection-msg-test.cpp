/*
 * Created By: Cameron Newton
 * Created On: May 5th, 2018
 * Description: A test to confirm that LineDetectMsg will build
 */

#include <gtest/gtest.h>
#include <line_detect/LineDetectMsg.h>

TEST(gate_detect, addMessage) {
    line_detect::LineDetectMsg msg;

    msg.lateralDistanceFromFrontMarker = 0.0;
    msg.lateralDistanceFromRearMarker  = 0.0;
    msg.angleToParallelFrontMarker     = 0.0;
    msg.angleToParallelRearMarker      = 0.0;
    msg.distanceFromEndOfFrontMarker   = 0.0;
    msg.distanceFromEndRearMarker      = 0.0;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
