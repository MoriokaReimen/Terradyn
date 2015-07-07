/**
-----------------------------------------------------------------------------
@file    wheel_soil.cpp
----------------------------------------------------------------------------
         @@
       @@@@@@
      @```@@@@
     @`  `@@@@@@
   @@`   `@@@@@@@@
  @@`    `@@@@@@@@@           Tohoku University
  @` `   `@@@@@@@@@       SPACE ROBOTICS LABORATORY
  @`` ## `@@@@@@@@@    http://www.astro.mech.tohoku.ac.jp/
  @` #..#`@@@@@@@@@        Planetary Robotics Group
  @` #..#`@@@@@@@@@
  @` ### `@@@@@@@@@          Professor Kazuya Yoshida
  @` ###``@@@@@@@@@      Associate Professor Keiji Nagatani
   @### ``@@@@@@@@
   ###  ` @@@@@@@
  ###  @  @@@@@                 Creation Date:
 ###    @@@@@               @date Dec. 16. 2014
 /-\     @@
|   |      %%                      Authors:
 \-/##    %%%%%             @author Kei Nakata
   #### %%%                  menschenjager.mark.neun@gmail.com
     ###%%       *
      ##%%     *****
       #%%      ***
        %%     *   *
         %%
          %%%%%
           %%%
-----------------------------------------------------------------------------
@brief Encapsulates Bekker-Wong Theory for Vehicle-Wheel Interactionshear
-----------------------------------------------------------------------------
*/
#include "wheel_soil.hpp"

using std::cos;
using std::sin;
using std::exp;
using std::bind;
using std::placeholders::_1;

/*
*    @brief Constructor of WheelSoil
*    @param [in] soil soil parameters
*    @param [in] wheel wheel parameters
*/
WheelSoil::WheelSoil(const Soil& soil, const Wheel& wheel) : soil_ {soil}, wheel_ {wheel} {
}

/*
*    @brief Get the normal stress along the front of the wheel for a given
*    @param [in] theta angle[radian]
*    @param [in] theta1 entry angle[radian]
*    @return radial stress in the front region of the soil[Pa]
*/
double WheelSoil::getSigma1_(const double& theta, const double& theta1) const
{
    double z = (cos(theta) - cos(theta1)) * wheel_.r; //! sinkage
    double sigma1 = (soil_.k1 + soil_.k2 * wheel_.b) * pow(z / wheel_.b, soil_.n);
    return sigma1;
}

/*
*    @brief Get the normal stress along the back part of the wheel
*    @param [in] theta normal stress on the sheared surface
*    @param [in] theta1 entry angle[radian]
*    @param [in] theta2 exit angle[radian]
*    @param [in] theta_m angular position of the maximum radial stress[radian]
*    @return radial stress in the rear region of the soil
*/
double WheelSoil::getSigma2_(const double& theta, const double& theta1, const double& theta2,
                             const double& theta_m) const
{
    double z = (cos(theta1 - ((theta - theta2) / (theta_m - theta2)) * (theta1 - theta_m))
                - cos(theta1)) * wheel_.r;
    double sigma_2 = soil_.k1 * pow(z / wheel_.b, soil_.n); //! k1 is k_eq here
    return sigma_2;
}

/*
*    @brief Get the normal stress at any angle on the wheel
*    @param [in] theta normal stress on the sheared surface
*    @param [in] theta1 entry angle[radian]
*    @param [in] theta2 exit angle[radian]
*    @param [in] theta_m angular position of the maximum radial stress[radian]
*    @return radial stress in the any region of the soil
*/
double WheelSoil::getSigma(const double& theta, const double& theta1, const double& theta2,
                           const double& theta_m) const
{
    double sigma {0.0};
    if(theta > theta_m && theta <= theta1)
        sigma = getSigma1_(theta, theta1);
    else if(theta >= theta2 && theta < theta_m)
        sigma = getSigma2_(theta, theta1, theta2, theta_m);

    return sigma;
}

/*
*    @brief Get the shear stress at any angle on the wheel lateral direction
*    @param [in] theta normal stress on the sheared surface
*    @param [in] theta1 entry angle[radian]
*    @param [in] theta2 exit angle[radian]
*    @param [in] theta_m angular position of the maximum radial stress[radian]
*    @param [in] slip  slip ratio
*    @return radial stress in the any region of the soil
*/
double WheelSoil::getTau_x(const double& theta, const double& theta1, const double& theta2,
                         const double& theta_m, const double& slip) const
{
    double tau {0.0};
    if(theta >= theta2 && theta <= theta1) {
        double sigma = getSigma(theta, theta1, theta2, theta_m);
        double j = ((theta1 - theta) - (1 - slip) * (sin(theta1) - sin(theta))) * wheel_.r;
        tau = (soil_.c + sigma * tan(soil_.phi)) * (1 - exp(-j / soil_.K));

    }
    return tau;
}

/*
*    @brief Get the shear stress at any angle on the wheel lateral direction
*    @param [in] theta normal stress on the sheared surface
*    @param [in] theta1 entry angle[radian]
*    @param [in] theta2 exit angle[radian]
*    @param [in] theta_m angular position of the maximum radial stress[radian]
*    @param [in] slip  slip ratio
*    @param [in] beta  slip ratio
*    @return radial stress in the any region of the soil
*/
double WheelSoil::getTau_y(const double& theta, const double& theta1, const double& theta2,
                         const double& theta_m, const double& slip, const double& beta) const
{
    double tau {0.0};
    if(theta >= theta2 && theta <= theta1) {
        double sigma = getSigma(theta, theta1, theta2, theta_m);
        double j = (wheel_.r * (1.0-slip) * (theta1-theta) * tan(beta));
        tau = (soil_.c + sigma * tan(soil_.phi)) * (1.0 - exp(-j / soil_.K));

    }
    return tau;
}

/*
*    @brief Get the Torque on the wheel
*    @param [in] theta1 entry angle[radian]
*    @param [in] theta2 exit angle[radian]
*    @param [in] theta_m angular position of the maximum radial stress[radian]
*    @param [in] slip  slip ratio
*    @return radial stress in the any region of the soil
*/
double WheelSoil::getWheelTorque( const double& theta1, const double& theta2, const double& theta_m, const double& slip) const
{
    auto tau_func = bind(&WheelSoil::getTau_x, this, _1, theta1, theta2, theta_m, slip);
    double torque = wheel_.b * wheel_.r * wheel_.r * integrate(tau_func, theta1, theta2);
    return torque;
}

/*
*    @brief Get the Drawbar pull on the wheel
*    @param [in] theta1 entry angle[radian]
*    @param [in] theta2 exit angle[radian]
*    @param [in] theta_m angular position of the maximum radial stress[radian]
*    @param [in] slip  slip ratio
*    @return drawbarpull of wheel
*/
double WheelSoil::getDrawbar(const double& theta1, const double& theta2, const double& theta_m, const double& slip) const
{
    auto sigma_buff = bind(&WheelSoil::getSigma, this, _1, theta1, theta2, theta_m);
    auto sigma_func = [&](double x) {
        return sigma_buff(x) * sin(x);
    };
    auto tau_buff = bind(&WheelSoil::getTau_x, this, _1, theta1, theta2, theta_m, slip);
    auto tau_func = [&](double x) {
        return tau_buff(x) * cos(x);
    };

    double drawbar = wheel_.r * wheel_.b * (integrate(tau_func, theta1, theta2) - integrate(sigma_func, theta1, theta2));
    return drawbar;
}

/*
*    @brief Get the Force on the wheel
*    @param [in] slip  slip ratio
*    @param [in] theta1 entry angle[radian]
*    @param [in] theta2 exit angle[radian]
*    @return force vector on the wheel
*/
Eigen::Vector3d WheelSoil::getForce(double slip, double theta1, double theta2) const
{
    Eigen::Vector3d force;
    double beta = this->getBeta();
    double fx, fy, fz;
    double theta_m = this->getTheta_m(slip, theta1);

    auto tau_x_buff = bind(&WheelSoil::getTau_x, this, _1, theta1, theta2, theta_m, slip);
    auto tau_y_buff = bind(&WheelSoil::getTau_y, this, _1, theta1, theta2, theta_m, slip, beta);
    auto sigma_buff = bind(&WheelSoil::getSigma, this, _1, theta1, theta2, theta_m);

    auto fx_func = [&](double x) {
        return tau_x_buff(x) * cos(x) - sigma_buff(x) * sin(x);
    };
    auto fy_func = [&](double x) {
        return tau_y_buff(x);
    };
    auto fz_func = [&](double x) {
        return sigma_buff(x) * cos(x) + tau_x_buff(x) * sin(x);
    };

    fx = (wheel_.r * wheel_.b) * integrate(fx_func, theta1, theta2);
    fy = (wheel_.r * wheel_.b) * integrate(fy_func, theta1, theta2);
    fz = (wheel_.r * wheel_.b) * integrate(fz_func, theta1, theta2);

    force(0) = fx;
    force(1) = fy;
    force(2) = fz - soil_.d * wheel_.velocity(2);

    return force;
}

/*
*    @brief Get the Torque on the wheel
*    @param [in] slip  slip ratio
*    @param [in] theta1 entry angle[radian]
*    @param [in] theta2 exit angle[radian]
*    @return torque vector on the wheel
*/
Eigen::Vector3d WheelSoil::getTorque(double slip, double theta1, double theta2) const
{
    Eigen::Vector3d torque;

    double theta_m = this->getTheta_m(slip, theta1);
    double beta = this->getBeta();

    auto tau_x_buff = bind(&WheelSoil::getTau_x, this, _1, theta1, theta2, theta_m, slip);
    auto tau_y_buff = bind(&WheelSoil::getTau_y, this, _1, theta1, theta2, theta_m, slip, beta);
    auto  tau_x = [&](double x) {
        return tau_x_buff(x);
    };
    auto  tau_y = [&](double x) {
        return tau_y_buff(x);
    };

    torque(0) = 0;
    torque(1) = wheel_.r * wheel_.r * wheel_.b * integrate(tau_x, theta1, theta2);
    torque(2) = wheel_.r * wheel_.r * wheel_.b * integrate(tau_y, theta1, theta2) * sin(theta_m); // self aligning torque

    return torque;
}

/*
*    @brief Get theta_m
*    @param [in] slip slip ratio
*    @param [in] theta1 entry angle[radian]
*    @return angle at max sigma[radian]
*/
double WheelSoil::getTheta_m(const double& slip, const double& theta1) const
{
    double theta_m = (soil_.a0 + soil_.a1 * slip) * theta1;
    return theta_m;
}

/*
*    @brief Get beta
*    @return slip angle[radian]
*/
double WheelSoil::getBeta() const
{
    double beta = std::min(fabs(asin(wheel_.velocity(1) / wheel_.velocity(0))), degToRad(45));
    return beta;
}
