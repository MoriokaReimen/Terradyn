//_/_/_/_/ vector library _/_/_/_//
#pragma once
#include <ode/ode.h>

class Vector3
{
    double x_;
    double y_;
    double z_;

public:
    Vector3();
    Vector3(const double x, const double y, const double z);
    void set(const double x, const double y, const double z);
    void setIdentity();
    void setZero();
    void scale(const double& s);
    void todReal(dReal* out);
    Vector3& operator+=(const Vector3& rhs);
    Vector3& operator-=(const Vector3& rhs);
friend Vector3 operator+(const Vector3& lhs, const Vector3& rhs);
friend Vector3 operator-(const Vector3& lhs, const Vector3& rhs);
friend double dot(const Vector3& lhs, const Vector3& rhs);
friend Vector3 cross(const Vector3& lhs, const Vector3& rhs);
};
