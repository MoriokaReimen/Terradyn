//_/_/_/_/ vector class _/_/_/_//
#include "vector.hpp"

Vector3::Vector3() : x_(0.0), y_(0.0), z_(0.0)
{
  return;
}

Vector3::Vector3(const double x, const double y, const double z)
{
  this->x_ = x;
  this->y_ = y;
  this->z_ = z;

  return;
}

void Vector3::set(const double x, const double y, const double z)
{
  this->x_ = x;
  this->y_ = y;
  this->z_ = z;

  return;
}

void Vector3::setIdentity()
{
  this->x_ = 1.0;
  this->y_ = 1.0;
  this->z_ = 1.0;

  return;
}

void Vector3::setZero()
{
  this->x_ = 0.0;
  this->y_ = 0.0;
  this->z_ = 0.0;

  return;
}
void Vector3::scale(const double& s)
{
  this->x_ = s * this->x_;
  this->y_ = s * this->y_;
  this->z_ = s * this->z_;

  return;
}

void Vector3::todReal(dReal* out)
{
  out[0] = static_cast<dReal>(this->x_);
  out[1] = static_cast<dReal>(this->y_);
  out[2] = static_cast<dReal>(this->z_);

  return;
}

Vector3& Vector3::operator+=(const Vector3& rhs)
{
  this->x_ += rhs.x_;
  this->y_ += rhs.y_;
  this->z_ += rhs.z_;

  return *this;
}

Vector3& Vector3::operator-=(const Vector3& rhs)
{
  this->x_ -= rhs.x_;
  this->y_ -= rhs.y_;
  this->z_ -= rhs.z_;

  return *this;
}

Vector3 operator+(const Vector3& lhs, const Vector3& rhs)
{
    Vector3 out;
    out.x_ = lhs.x_ + rhs.x_;
    out.y_ = lhs.y_ + rhs.y_;
    out.z_ = lhs.z_ + rhs.z_;

    return out;
}

Vector3 operator-(const Vector3& lhs, const Vector3& rhs)
{
    Vector3 out;
    out.x_ = lhs.x_ - rhs.x_;
    out.y_ = lhs.y_ - rhs.y_;
    out.z_ = lhs.z_ - rhs.z_;

    return out;
}

double dot(const Vector3& lhs, const Vector3& rhs)
{
    double sum(0.0);
    sum += lhs.x_ * rhs.x_;
    sum += lhs.y_ * rhs.y_;
    sum += lhs.z_ * rhs.z_;

    return sum;
}

Vector3 cross(const Vector3& lhs, const Vector3& rhs)
{
    Vector3 out;
    out.x_ = lhs.y_*rhs.z_-lhs.z_*rhs.y_;
    out.y_ = lhs.z_*rhs.x_-lhs.x_*rhs.z_;
    out.z_ = lhs.x_*rhs.y_-lhs.y_*rhs.x_;
    return out;
}

/* implement later
void vector_tilde3( double *a, double *b )
{
    b[0] = 0;
    b[1] = -a[2];
    b[2] = a[1];
    b[3] = a[2];
    b[4] = 0;
    b[5] = -a[0];
    b[6] = -a[1];
    b[7] = a[0];
    b[8] = 0;
}
*/
