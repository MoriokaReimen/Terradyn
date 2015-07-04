//_/_/_/_/ vector class _/_/_/_//
#include "vector.hpp"

void todReal(dReal* out, Eigen::Vector3d vec)
{
  out[0] = vec.x();
  out[1] = vec.y();
  out[2] = vec.z();

  return;
}
