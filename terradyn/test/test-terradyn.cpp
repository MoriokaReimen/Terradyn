#include <gtest/gtest.h>
#include "Terradyn.hpp"
#include "terradyn.hpp"
#include <iostream>
#include <cmath>

TEST(WheelSoilTEST, getForce)
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

  auto force = wheel_soil.getForce(slip, theta1, theta2);

  EXPECT_NEAR(fe[0], force(0), 0.3);
  EXPECT_NEAR(fe[1], force(1), 0.3);
  EXPECT_NEAR(fe[2], force(2), 0.3);
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

  EXPECT_NEAR(0, torque(0), 0.3);
  EXPECT_NEAR(fe[4], torque(1), 0.3);
  EXPECT_NEAR(fe[3], torque(2), 0.3);
}

