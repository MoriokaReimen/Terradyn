#include <gtest/gtest.h>
#include "wheel_soil.hpp"
#include "terradyn.h"
#include <iostream>

TEST(WheelSoilTEST, getForce)
{
  Soil soil;
  Wheel wheel;
  wheel.velocity(0) = 0.0;
  wheel.velocity(1) = 2.0;
  wheel.velocity(2) = -1.0;
  WheelSoil wheel_soil(soil, wheel);
  double buff = wheel_soil.getTheta_m(0.6, toRadian(50));
  std::cout << toDegree(buff) << std::endl;
  Eigen::Vector3d force = wheel_soil.getForce(0.6, toRadian(20), toRadian(10));
  double a[5] ;
  EXPECT_NEAR(a[0], force(0), 0.01);
  EXPECT_NEAR(a[1], force(1), 0.01);
  EXPECT_NEAR(a[2], force(2), 0.01);
}

