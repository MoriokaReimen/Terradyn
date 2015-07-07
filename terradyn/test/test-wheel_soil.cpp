#include "wheel_soil.hpp"
#include <gtest/gtest.h>

TEST(WheelSoilTEST, getForce)
{
  EXPECT_NEAR(21.333333, integrate(func, 0, 4), 0.01);
}

TEST(WheelSoilTEST, getTorque)
{
  EXPECT_NEAR(21.333333, integrate(func, 0, 4), 0.01);
}
