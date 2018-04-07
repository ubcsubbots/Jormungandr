/*
 *  Author: Joel
 *  Date:  31/03/18
 *  Purpose: Unit test to instantiate and message correctly outputs
 */
#include <worldstate/state_msg.h>
#include <gtest/gtest.h>

TEST(worldState, instantiateMsg) {
    worldstate::state_msg msg;

    msg.state = worldstate::state_msg<uint8_t>::locatingGate;

    EXPECT_TRUE((msg.state == 0));

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
