/**
-----------------------------------------------------------------------------
@file    wheel_soil.hpp
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
#pragma once
#include<cmath>
#include <functional>
#include <eigen3/Eigen/Core>

#include "angle.hpp"
#include "gsl.hpp"

using std::function;

/*
*    @struct Soil
*    @brief parameters for Soil
*/
struct Soil {
    double phi {toRadian(38.0)}; //! angle of internal shearing resistance
    double c {0.0};               //! cohesion[Pa]
    double k_x{0.014};            //! shear deformation modulus[m]
    double k_y{0.016};            //! shear deformation modulus[m]
    double k_c{0.0};
    double k_phi{1203.54};
    double n {1.703};                  //! exponent of sinkage to width ratio
    double d{1.0}; // sink constant[m/]
    double a0{0.4}; // max sigma angle constant1
    double a1{0.15}; //max sigma angle constant2
};

/*
*    @struct Wheel
*    @brief parameters for wheel
*/
struct Wheel {
    /* Wheel properties */
    double r {0.055};                   //! radius[m]
    double b {0.064};                   //! width[m]
    Eigen::Vector3d velocity;
};

/*
*    @class Terradyn
*    @brief Encapsulates Bekker-Wong Theory for Vehicle-Wheel Interaction
*/
class Terradyn
{
private:
    Soil soil_;
    Wheel wheel_;
    double getSigma1_(const double& theta, const double& theta1) const;
    double getSigma2_(const double& theta, const double& theta1, const double& theta2,
                      const double& theta_m) const;
public:
    Terradyn(const Soil& soil, const Wheel& wheel);
    double getSigma(const double& theta, const double& theta1, const double& theta2,
                    const double& theta_m) const;
    double getTau_x(const double& theta, const double& theta1, const double& theta2,
                  const double& theta_m, const double& slip) const;
    double getWheelTorque(const double& theta1, const double& theta2, const double& theta_m, const double& slip) const;
    double getDrawbar(const double& theta1, const double& theta2, const double& theta_m, const double& slip) const;
    double getTau_y(const double& theta, const double& theta1, const double& theta2, const double& theta_m, const double& slip, const double& beta) const;
    Eigen::Vector3d getForce(double slip, double theta1, double theta2) const;
    Eigen::Vector3d getTorque(double slip, double theta1, double theta2) const;
    double getTheta_m(const double& slip, const double& theta1) const;
    double getBeta() const;
    double getTau_max(const double& theta, const double& theta1, const double& theta2,
                    const double& theta_m) const;
};
