#include <gtest/gtest.h>
#include "wheel_soil.hpp"
#include "terradyn.h"
#include <iostream>

TEST(WheelSoilTEST, getForce)
{
  Soil soil;
  Wheel wheel;
  wheel.velocity(0) = 1.0;
  wheel.velocity(1) = 1.0;
  wheel.velocity(2) = -1.0;
  WheelSoil wheel_soil(soil, wheel);
  EXPECT_NEAR(1.0, wheel.velocity(0), 0.01);
}

