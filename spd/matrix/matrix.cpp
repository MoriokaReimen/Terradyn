#include "matrix.hpp"

void todReal(dReal* out, Eigen::Matrix3d mat)
{
  out[0] = static_cast<dReal>(mat(0, 0));
  out[1] = static_cast<dReal>(mat(0, 1));
  out[2] = static_cast<dReal>(mat(0, 2));
  out[3] = static_cast<dReal>(mat(1, 0));
  out[4] = static_cast<dReal>(mat(1, 1));
  out[5] = static_cast<dReal>(mat(1, 2));
  out[6] = static_cast<dReal>(mat(2, 0));
  out[7] = static_cast<dReal>(mat(2, 1));
  out[8] = static_cast<dReal>(mat(2, 2));

  return;
}
