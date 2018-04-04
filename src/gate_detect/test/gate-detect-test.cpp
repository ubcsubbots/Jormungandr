//
// Created by da-cam on 03/04/18.
//

#include <gate_detect/gateDetectMsg.h>
#include <gtest/gtest.h>

TEST(gate_detect, addMessage) {
    gate_detect::gateDetectMsg msg;

    msg.detectLeft = 1;
    msg.angleLeft = 2.0f;
    msg.distanceLeft = 2.0f;
    msg.detectRight = 1;
    msg.angleRight = 2.0f;
    msg.distanceRight = 2.0f;
    msg.detectTop = 1;
    msg.angleTop = 2.0f;
    msg.distanceTop = 2.0f;

    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}