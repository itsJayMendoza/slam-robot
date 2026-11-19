#include <gtest/gtest.h>

#include <cmath>

#include "slam_robot_hardware/protocol.hpp"

using slam_robot_hardware::fault_text;
using slam_robot_hardware::parse_state;
using slam_robot_hardware::velocity_command;

TEST(Protocol, ParsesState)
{
  auto s = parse_state("e 358243 -358550 17228 17908 0");
  ASSERT_TRUE(s);
  EXPECT_EQ(s->left, 358243);
  EXPECT_EQ(s->right, -358550);
  EXPECT_DOUBLE_EQ(s->left_cps, 17228);
  EXPECT_DOUBLE_EQ(s->right_cps, 17908);
  EXPECT_EQ(s->fault, 0);
}

TEST(Protocol, RejectsOtherLines)
{
  for (const char * line : {"ver slam-robot-base 0.4", "e 1 2 3", "e a b c d e", "ok stopped", "",
      "e 1 2 3 4 5 6", "EN pin"})
  {
    EXPECT_FALSE(parse_state(line)) << line;
  }
}

TEST(Protocol, VelocityRoundsAndClamps)
{
  EXPECT_EQ(velocity_command(1234.6, -99.4, 50000), "v 1235 -99");
  EXPECT_EQ(velocity_command(90000, -90000, 50000), "v 50000 -50000");
  EXPECT_EQ(velocity_command(NAN, 0, 50000), "v 0 0");
}

TEST(Protocol, FaultText)
{
  EXPECT_EQ(fault_text(0), "none");
  EXPECT_EQ(fault_text(17), "left stalled, left overspeed");
}
