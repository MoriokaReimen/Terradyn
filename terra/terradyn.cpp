//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Terra mehcanics + Dynamics = Terra dyn
// Function file
//
// G.Ishigami [2005.10]
//       -add negative slip version Nov.15th
//
// modified by INO [2012.12]
//       - extended to inclined wheels
// modified by Kei [2015.7]
//       - C++ 11
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
#include<cstdio>
#include<cmath>

#include"terradyn.hpp"

#define _USE_MATH_DEFINES

double cot(const double& radian)
{
  return std::tan(M_PI_2 - radian);
}

double deg2rad(const double& degree)
{
  return degree*M_PI/180.0;
}

double rad2deg(const double& radian)
{
  return radian/M_PI*180.0;
}


double Dz = 1.0;



int sgn2(double val)
{
  if(val>0)
    {   return  1;
    }
    else if(val<0)
    {   return -1;
    }
    return 0;
}


/* Caclculate effective wheel width *//////////////////////////////////////////////
void calcWidth(double roll, double h0)
{
  if(fabs(roll) < 0.01)
    {   w_b_eff = w_b;
    }
    else
    {   w_b_eff = h0*cot(roll) + w_b/2.0;
        if(w_b_eff > w_b)
        {   w_b_eff = w_b;
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////


/* Calculate lateral sinkage distribution *////////////////////////////////////////
double calcSinkage(double roll, double h0, double y)
{
    double h;
    h = (y*tan(roll)+h0);
    if (h > 0.0)
    {   return h;
    }
    else
    {
      return 0.0;
    }
}
///////////////////////////////////////////////////////////////////////////////////


/* Calculate lateral normal stress distribution *//////////////////////////////////
double calc_sigma_inclined(double theta, double theta_f, double theta_r, double theta_m)
{
    double k_sigma;

    // Normal stress coeffiecient
    k_sigma = (SOIL_C*SOIL_KC+SOIL_GAMMA*w_b_eff*SOIL_KPHI)*pow(w_rad/w_b_eff, SOIL_N);

    if((theta>=theta_m) && (theta<theta_f))
    {   return k_sigma*pow(cos(theta)-cos(theta_f), SOIL_N);
    }
    else if((theta>theta_r) && (theta<theta_m))
    {   return k_sigma*pow(cos(theta_f-(theta-theta_r)/(theta_m-theta_r)*(theta_f-theta_m))-cos(theta_f), SOIL_N);
    }
    else
    {   return 0.0;
    }
}
///////////////////////////////////////////////////////////////////////////////////


/* Calculate shear stresses *//////////////////////////////////////////////////////
void calcTau(double s, double beta, double theta, double theta_f, double sigma, double tau[])
{   double eta, ajt, ajl, j, jt, jl;

    // Soil slip angle
    ajt = 1-(1-s)*cos(theta);
    ajl = (1-s)*tan(beta);
    eta = atan2(ajl, ajt);

    // Soil deformation
    jt = w_rad*((theta_f-theta) - (1-s)*(sin(theta_f)-sin(theta)));
    jl = w_rad*(1-s)*(theta_f-theta)*tan(beta);
    j = sqrt(jt*jt + jl*jl);

    // Shear stresses
    tau[0] = (SOIL_C + sigma*tan(SOIL_PHI))*(1-exp(-j/SOIL_K));	// tau_total
    tau[1] = tau[0]*cos(eta);			// tau_t
    tau[2] = tau[0]*sin(eta);			// tau_l
}
///////////////////////////////////////////////////////////////////////////////////


/* Calculate dFxb, dFyb, dFzb, dTz, dTx *//////////////////////////////////////////
void calc_dFb(double s, double beta, double h, double dFb[])
{   double theta_f, theta_r, theta_m;
    double theta;
    double h_theta;
    double sigma, tau[3];
    double dFx=0.0, dFy=0.0, dFz=0.0, dTz=0.0;

    kappa = 2.1033*s*s - 1.9494*s + 0.8151;
    theta_f = acos(1-h/w_rad);
    theta_r = -acos(1-kappa*h/w_rad);
    theta_m = (SOIL_A0+SOIL_A1*s)*theta_f;

    // Integrands of dFx, dFy, dFz (trapezoid formula)
    for (theta=theta_r; theta<=theta_f; theta += DELTA_THETA)
    {   // Sinkage
        h_theta = w_rad*(cos(theta)-cos(theta_f));

        // Stresses
        sigma = calc_sigma_inclined(theta, theta_f, theta_r, theta_m);
        calcTau(s, beta, theta, theta_f, sigma, tau);

        dFx += tau[1]*cos(theta) - sigma*sin(theta);
        dFy += tau[2];
        dFz += tau[1]*sin(theta) + sigma*cos(theta);
        dTz += tau[1];
    }
    dFb[0] = w_rad * dFx * DELTA_THETA;  // dFx [N/m]
    dFb[1] = w_rad * dFy * DELTA_THETA;  // dFy [N/m]
    dFb[2] = w_rad * dFz * DELTA_THETA;  // dFz [N/m]
    dFb[3] = w_rad * w_rad * dTz * DELTA_THETA;  // dTz [(N*m)/m]
    dFb[4] = w_rad * dFb[1] * theta_m;   // dTx [(N*m)/m]
}
///////////////////////////////////////////////////////////////////////////////////


/* Calculate Fxb, Fyb, Fzb, Tz, Tx *///////////////////////////////////////////////
void calc_Fb(double s, double beta, double roll, double theta_f0, double theta_r0, double Fb[])
{   double h0, h;
    double y;
    double dFb[5]= {};

    for(int i=0; i<5; i++)
    {   dFb[i] = 0.0;
    }

    // Total sinkage at y=0
    h0 = w_rad*(1-cos(theta_f0));

    // Effective wheel width
    calcWidth(roll, h0);

    for(y=-w_b/2.0; y<=w_b/2.0; y+=DELTA_Y)
    {   // Forces at position y
        h = calcSinkage(roll, h0, y);
        calc_dFb(s, beta, h, dFb);
        for(int i=0; i<5; i++)
        {   Fb[i] += dFb[i]*DELTA_Y; // Fxb, Fyb, Fzb, Tz, Tx
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////


/* Calculate initial sinkage at y=0 *//////////////////////////////////////////////
double calcInitSinkage(double W, double roll)
{
    double N, e;
    double K_tmp, K, K_dush, integral, integral_dush;
    double theta_fc0, theta_fc, theta_r;
    double h, h0;
    double high, low;
    double delta2;
    int count=0;

    theta_fc0 = 45.0 * M_PI / 180;
    high = theta_fc0 * 2;
    low = 0;
    e = 10000;
    delta2 = delta - roll;

    while(fabs(e) > 0.01)
    {

        N=0.0;
        integral = 0.0;
        integral_dush = 0.0;

        h0 = w_rad*(1-cos(theta_fc0));
        calcWidth(roll, h0);

        for(double y=-w_b/2.0; y<=w_b/2.0; y+=DELTA_Y)
        {   h = calcSinkage(roll, h0, y);
            theta_fc = acos(1-h/w_rad);
            theta_r = 2 * alpha - theta_fc;

            K_tmp = (SOIL_C*SOIL_KC + SOIL_GAMMA*SOIL_KPHI*w_b_eff) * pow(w_rad, (SOIL_N+1))/pow(w_b_eff, SOIL_N);
            K = K_tmp / ( pow(cos(alpha),SOIL_N) * pow(cos(delta2),(SOIL_N-1)) );

            for(double theta=theta_r+DELTA_THETA; theta<theta_fc; theta += DELTA_THETA)
            {   integral += pow( cos(theta - alpha)-cos(theta_fc - alpha), SOIL_N) * cos(theta);
                //if(integral>100) {break;}
            }
        }
        N = K * integral * DELTA_THETA * DELTA_Y;

        if(fabs(delta2) > 0.01)
        {   if(roll > 0)
            {   h = calcSinkage(roll, h0, -w_b/2.0);
            }
            else
            {   h = calcSinkage(roll, h0, w_b/2.0);
            }
            theta_fc = acos(1-h/w_rad);
            theta_r = 2*alpha - theta_fc;
            for(double theta=theta_r+DELTA_THETA; theta<theta_fc; theta += DELTA_THETA)
            {   integral_dush += pow((cos(theta - alpha) - cos(theta_fc - alpha)) , SOIL_N) * pow(sin(theta-alpha),2);
            }
            K_dush = K_tmp * w_rad * fabs(sin(delta2)) / pow(cos(delta2),SOIL_N);
            N += K_dush * integral_dush * DELTA_THETA;
        }

        //e = W * cos(alpha) * cos(alpha) - N; // why multiplying by cos^2?
        e = W - N;

        if(e > 0.0)
        {   low = theta_fc0;
            theta_fc0 = (theta_fc0 + high) / 2;
            if(theta_fc0 > deg2rad(90))
                theta_fc0 = deg2rad(89);
        }
        else
        {   high = theta_fc0;
            theta_fc0 = (theta_fc0 + low) / 2;
            if(theta_fc0 < 0.0)
                theta_fc0 = 0.0;
        }
        count++;
        //if(count > MAX_CALC) {break;}
    }
    return theta_fc0;
}
///////////////////////////////////////////////////////////////////////////////////


/* Calculate wheel forces *////////////////////////////////////////////////////////
void tCalc_Fe_positive2(double s, double beta, double theta_f0, double theta_r0,
                        double vz, double PHI, double *fe, double roll)
{   double Fb[5]= {}, Fs[3]= {};

    for(int i=0; i<5; i++)
    {   fe[i] = 0.0;
        Fb[i] = 0.0;
        if(i<3)
        {   Fs[i]=0.0;
        }
    }

    // adapt theta into ground coordinate
    theta_f0 -= PHI;
    theta_r0 -= PHI;

    calc_Fb(s, fabs(beta), roll, theta_f0, theta_r0, Fb);
    //calc_Fs(s, delta, roll, theta_f0, Fs);
    fe[0] = Fb[0] + Fs[0];		// Fx
    //fe[1] = Fb[1]*sgn2(-beta) + Fs[1];	// Fy
    fe[1] = Fb[1] + Fs[1];	// Fy
    fe[2] = Fb[2] + Fs[2] - Dz*vz;	// Fz
    fe[3] = Fb[3];			// Tz
    fe[4] = Fb[4];			// Tx
}
///////////////////////////////////////////////////////////////////////////////////


