#include "wheel_soil.hpp"
#include "terradyn.h"
#include <gtest/gtest.h>

TEST(WheelSoilTEST, getForce)
{
  Soil soil;
  Wheel wheel;
  WheelSoil wheel_soil(wheel, soil);
  auto force = wheel_soil.getForce();
  double fe[5];
  tCalc_Fe_positive(0.3, 0.1, toRadian(40), toRadian(-20), -1.0, 0.0, fe);
  EXPECT_NEAR(fe[0], force(0), 0.01);
  EXPECT_NEAR(fe[1], force(1), 0.01);
  EXPECT_NEAR(fe[2], force(2), 0.01);
}

/*
TEST(WheelSoilTEST, getTorque)
{
  EXPECT_NEAR(21.333333, integrate(func, 0, 4), 0.01);
}
*/
