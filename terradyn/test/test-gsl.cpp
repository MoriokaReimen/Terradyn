#include "gsl.hpp"
#include <gtest/gtest.h>

TEST(GSL_Test, integrate)
{
  auto func = [](double x){return x * x;};
  EXPECT_NEAR(21.333333, integrate(func, 0, 4), 0.01);
}

TEST(GSL_Test, differentiate)
{
  auto func = [](double x){return x * x * x;};
  EXPECT_NEAR(12.0, differentiate(func, 2), 0.01);
}

TEST(GSL_Test, find_root1)
{
  auto func = [](double x){return x * x * x - 4;};
  EXPECT_NEAR(1.5874010, find_root(func), 0.01);
}

TEST(GSL_Test, find_root2)
{
  auto func = [](double x){return x * x * x;};
  EXPECT_NEAR(1.5874010, find_root(func, 4.0), 0.01);
}
