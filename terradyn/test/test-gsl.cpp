#include "RotateFunc.hpp"

#include <gtest/gtest.h>
using namespace Math3D;

TEST(RotateFuncTest, rotate_func)
{
  Vector3 x(1.0, 0.0, 0.0);
  Vector3 y(0.0, 1.0, 0.0);
  Vector3 z(0.0, 0.0, 1.0);
  Degree angle(90.0);

  Quaternion quat_a(angle, z);
  auto ans_a = rotate(quat_a, x);
  EXPECT_NEAR(0.0, ans_a.x, 0.01);
  EXPECT_NEAR(1.0, ans_a.y, 0.01);
  EXPECT_NEAR(0.0, ans_a.z, 0.01);

  Quaternion quat_b(angle, x);
  auto ans_b = rotate(quat_b, y);
  EXPECT_NEAR(0.0, ans_b.x, 0.01);
  EXPECT_NEAR(0.0, ans_b.y, 0.01);
  EXPECT_NEAR(1.0, ans_b.z, 0.01);
}

