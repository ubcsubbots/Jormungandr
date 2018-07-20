/*
 * Created By: Cameron Newton
 * Created On: May 5th, 2018
 * Description: A test to confirm that gateDetectMsg will build
 */

#include <gate_detect/GateDetectMsg.h>
#include <gtest/gtest.h>

TEST(gate_detect, addMessage) {
    gate_detect::GateDetectMsg msg;

    msg.detectedLeftPole  = 1;
    msg.angleLeftPole     = 2.0f;
    msg.distanceLeftPole  = 2.0f;
    msg.detectedRightPole = 1;
    msg.angleRightPole    = 2.0f;
    msg.distanceRightPole = 2.0f;
    msg.detectedTopPole   = 1;
    msg.angleTopPole      = 2.0f;
    msg.distanceTopPole   = 2.0f;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
