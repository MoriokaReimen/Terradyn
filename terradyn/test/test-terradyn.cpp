#include <gtest/gtest.h>
#include "Terradyn.hpp"
#include "terradyn.hpp"
#include <iostream>
#include <cmath>

TEST(WheelSoilTEST, getForce)
{
  Soil soil;
  Wheel wheel;
  wheel.r = 0.055;
  wheel.b = 0.064;
  wheel.velocity(0) = 5.0;
  wheel.velocity(1) = 2.0;
  wheel.velocity(2) = -1.0;

  Terradyn wheel_soil(soil, wheel);
  double beta = wheel_soil.getBeta();
  double slip = 0.9;
  double theta1 = toRadian(20);
  double theta2 = toRadian(10);
  double fe[5];

  tCalc_Fe_positive(slip, beta, theta1, theta2, wheel.velocity(2), 0.0, fe);

  auto force = wheel_soil.getForce(slip, theta1, theta2);

  EXPECT_NEAR(0.0, force(0)/fe[0] - 1, 0.15);
  EXPECT_NEAR(0.0, force(1)/fe[1] - 1, 0.15);
  EXPECT_NEAR(0.0, force(2)/fe[2] - 1, 0.15);
}

TEST(WheelSoilTEST, getTorque)
{
  Soil soil;
  Wheel wheel;
  wheel.velocity(0) = 4.0;
  wheel.velocity(1) = 2.0;
  wheel.velocity(2) = -1.0;

  Terradyn wheel_soil(soil, wheel);
  double beta = wheel_soil.getBeta();
  double slip = 0.6;
  double theta1 = toRadian(20);
  double theta2 = toRadian(10);
  double fe[5];

  tCalc_Fe_positive(slip, beta, theta1, theta2, -1.0, 0.0, fe);

  auto torque = wheel_soil.getTorque(slip, theta1, theta2);

  EXPECT_NEAR(0, torque(0), 0.15);
  EXPECT_NEAR(0.0, torque(1)/fe[3] -1.0, 0.15);
  EXPECT_NEAR(0.0, torque(2)/fe[4] -1.0, 0.15);
}

