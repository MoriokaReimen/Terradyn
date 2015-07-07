#include "gsl.hpp"
#include <gtest/gtest.h>

TEST(GSL_Test, integrate)
{
  auto func = [](double x){return x * x;};
  EXPECT_NEAR(21.333333, integrate(func, 0, 4), 0.01);
}

