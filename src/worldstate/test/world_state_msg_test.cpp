/*
 *  Author: Joel
 *  Date:  31/03/18
 *  Purpose: Unit test to instantiate and message correctly outputs
 */
#include <gtest/gtest.h>
#include <worldstate/state_msg.h>

TEST(worldState, instantiateMsg) {
    worldstate::state_msg msg;

    msg.state = worldstate::state_msg_<u_int8_t>::locatingGate;
    EXPECT_EQ(msg.state, 0);

    msg.state = worldstate::state_msg_<u_int8_t>::aligningWithGate;
    EXPECT_EQ(msg.state, 1);

    msg.state = worldstate::state_msg_<u_int8_t>::passingGate;
    EXPECT_EQ(msg.state, 2);

    msg.state = worldstate::state_msg_<u_int8_t>::locatingPole;
    EXPECT_EQ(msg.state, 3);

    msg.state = worldstate::state_msg_<u_int8_t>::approachingPole;
    EXPECT_EQ(msg.state, 4);

    msg.state = worldstate::state_msg_<u_int8_t>::pivotingPole;
    EXPECT_EQ(msg.state, 5);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
