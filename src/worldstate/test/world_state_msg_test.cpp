/*
 *  Author: Joel
 *  Date:  31/03/18
 *  Purpose: Unit test to instantiate worldstate StateMsg's and ensure
 *           they are built correctly
 */

#include <gtest/gtest.h>
#include <worldstate/StateMsg.h>

TEST(CheckStateValuesMatch, instantiateMsg) {
    worldstate::StateMsg msg;

    msg.state = worldstate::StateMsg_<u_int8_t>::locatingGate;
    EXPECT_EQ(msg.state, 0);

    msg.state = worldstate::StateMsg_<u_int8_t>::aligningWithGate;
    EXPECT_EQ(msg.state, 1);

    msg.state = worldstate::StateMsg_<u_int8_t>::passingGate;
    EXPECT_EQ(msg.state, 2);

    msg.state = worldstate::StateMsg_<u_int8_t>::searchingForPath;
    EXPECT_EQ(msg.state, 3);

    msg.state = worldstate::StateMsg_<u_int8_t>::locatingDie;
    EXPECT_EQ(msg.state, 4);

    msg.state = worldstate::StateMsg_<u_int8_t>::touchingDie;
    EXPECT_EQ(msg.state, 5);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
