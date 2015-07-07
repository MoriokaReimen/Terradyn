//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Terra mehcanics + Dynamics = Terra dyn
// Function file
//            
// G.Ishigami [2005.10]
//       -add negative slip version Nov.15th
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
#include<stdio.h>
#include<math.h>

#include"terradyn.h"

#define sqr_f(x) x*x
#define cot(x) 1/tan(x)
#define deg2rad(x) x*M_PI/180.0
#define rad2deg(x) x/M_PI*180.0

static double Dz = 1.0;
  
double calc_jx(double s, double theta_f, double theta)
{
  return (w_rad*(theta_f - theta - (1.0-s) * (sin(theta_f) - sin(theta))));
}

double calc_jy(double s, double theta_f, double theta, double beta)
{
  return (w_rad * (1.0-s) * (theta_f-theta) * tan(beta));
}

double calc_Rb(double H)
{
  double D1, D2;
  double Xc;
  double h0;
  
  Xc = 45.0*M_PI/180.0 - 0.5*phi;
  D1 = cot(Xc) + tan(Xc + phi);
  D2 = cot(Xc) + ( pow(cot(Xc), 2) / cot(phi) );
  h0 = H*cot(Xc)/cot(phi); 
  H += h0;
  //*/

  //ver_new 2005 11 16 
  /*  double i;
  i = delta;
  Xc = 45.0*M_PI/180.0 - 0.5*phi + 0.5*(i + asin( (sin(i)/sin(phi))) );
  D1 = cot(Xc) + tan((Xc + phi - fabs(i) ));
  D2 = cot(Xc) + ( pow(cot(Xc), 2) / cot((phi - fabs(i) )) );
  h0 = H*cot(Xc)/cot(phi); 
  H += h0;
  //*/
  return D1 * ( H*c + 0.5*rho* pow(H,2) * D2);
}

double calc_theta_t(double s)
{
  double a, b;
  a = 45.0*M_PI/180.0 - 0.5*phi;
  b = 1.0/(1.0 - s);
  
  return acos( (b + sqrt( sqr_f(b) + (1.0+sqr_f(a))*(sqr_f(a)-sqr_f(b)))) / (1+sqr_f(a)));
}

double calc_Kv(double s, double theta_f, double theta_t)
{
  return (1.0 / (1.0 - s)) * ( ((1.0-s)*(sin(theta_f)-sin(theta_t)) / (theta_f -theta_t)) -1.0 );
}

double calc_jxf(double s, double theta_f, double theta, double Kv)
{
  return w_rad*( (theta_f-theta)*(1.0+(1.0-s)*Kv) - (1.0-s)*(sin(theta_f)-sin(theta)) );
}

double tInit_sinkage(double W)
{
  double i, e, N, K_tmp;
  double K,K_dush, integral, integral_dush;
  double theta_fc, theta_r, high, low, d_theta;

  if(K_flag==0)
    // bekker
    K_tmp = (k_c + k_phi * w_b) * pow(w_rad, (n+1));
  else 
    // Reece
    K_tmp = (c*k_c + rho*k_phi*w_b) * pow(w_rad, (n+1))/pow(w_b, (n-1));

  K = K_tmp / ( pow(cos(alpha),n) * pow(cos(delta),(n-1)) );
  K_dush = K_tmp* w_rad * fabs(sin(delta)) / pow(cos(delta),n);
  
  theta_fc = 45.0 * M_PI / 180;  
  high = theta_fc * 2;
  low = 0;
  d_theta = 0.001;
  e = 10000;

  //while(fabs(e) > 0.0000001){
  while(fabs(e) > 0.01){
      
    N=0.0;
    integral = 0.0;
    integral_dush = 0.0;
    theta_r = 2 * alpha - theta_fc;
    
    for(i=theta_r+d_theta; i<theta_fc; i += d_theta){
      integral += pow( cos(i - alpha)-cos(theta_fc - alpha), n) * cos(i) * d_theta;
    
      if(integral>100)
	break;
      
      if(delta != 0.0)
	integral_dush += pow((cos(i - alpha) - cos(theta_fc - alpha)) , n) * pow(sin(i-alpha),2) * d_theta;
    }
    
    N = K * integral;
    
    if(delta != 0)
      N = N + K_dush * integral_dush;
    
    e = W * cos(alpha) * cos(alpha) - N;
    
    if(e > 0.0){
      low = theta_fc;
      theta_fc = (theta_fc + high) / 2;
      if(theta_fc > deg2rad(90))
	theta_fc = deg2rad(89);
    }
    else{
      high = theta_fc;
      theta_fc = (theta_fc + low) / 2;
      if(theta_fc < 0.0)
	theta_fc = 0.0;
      
    }
  }
  printf("%lf\n",N);
  return theta_fc;
}	


void tCalc_Fe_positive(double s, double beta, double theta_f, double theta_r,  double vz, double PHI, double *fe)
{
  double i, K, Fx, Fu, Fs, Fz, H, Rb;
  double theta_m, d_theta, Tx, Ty, Tz;
  double sigma, tau_x, tau_y, tau_max, jx, jy;
  double kx, ky;

  Fz = 0.0;
  Fx = 0.0;
  Fu = 0.0;
  Fs = 0.0;
  
  Tx = 0.0;
  Ty = 0.0;
  Tz = 0.0;

  sigma = 0.0;
  tau_x = 0.0;
  tau_y = 0.0;

  jx = 0.0;
  jy = 0.0;

  H  = 0.0;
  Rb = 0.0;  

  fe[0] = 0.0;
  fe[1] = 0.0;
  fe[2] = 0.0;
  fe[3] = 0.0;
  fe[4] = 0.0;
   
  // adapt theta into ground coordinate
  theta_f -= PHI;
  theta_r -= PHI;
  d_theta = deg2rad(0.5);//0.00;
  
  // the angle of max stress
  theta_m = (a_0 + a_1 * s) * theta_f;

  if(K_flag==0)
    //bekker
    K = (k_c / w_b + k_phi) * pow(w_rad,n);
  else
    //reece
    K = (c*k_c + rho*k_phi*w_b) * pow(w_rad/w_b, n);
    
  beta = fabs(beta);
  if(beta>deg2rad(45))
    beta  = deg2rad(45);
  
  kx =  -0.0073*beta*beta - 0.0403*beta + 0.07;
  ky = 0.0306667;
  
  for(i=theta_m; i<=theta_f; i+=d_theta){
    sigma = K * pow( (cos(i) - cos(theta_f)), n);
    
    tau_max = (c + sigma*tan(phi));

    jx = calc_jx(s, theta_f, i);
    jy = calc_jy(s, theta_f, i, beta);
    
    tau_x = tau_max * (1 - exp(-jx/kx));
    tau_y = tau_max * (1 - exp(-jy/ky));
    
    Fz += (sigma * cos(i) * d_theta + tau_x * sin(i) * d_theta);
    Fx += (tau_x * cos(i) * d_theta - sigma * sin(i) * d_theta);
    
    H = w_rad * (cos(i) - cos(theta_f));
    Rb = calc_Rb(H);
    
    Fu += (tau_y * d_theta);
    Fs += (Rb*(w_rad - H*cos(i))*d_theta);
    
    Tz += (tau_x * d_theta);
  }
  for(i=theta_r; i<theta_m; i+=d_theta){
    
    sigma = K * pow(cos(theta_f - (i-theta_r)/(theta_m-theta_r)*(theta_f-theta_m)) - cos(theta_f), n);

    if( (cos(theta_f - (i-theta_r)/(theta_m-theta_r)*(theta_f-theta_m)) - cos(theta_f)) < -0.1)
      printf("ARG is negative!! \n");

    tau_max = (c + sigma*tan(phi));
    
    jx = calc_jx(s, theta_f, i);
    jy = calc_jy(s, theta_f, i, beta);
    
    tau_x = tau_max * (1 - exp(-jx/kx));
    tau_y = tau_max * (1 - exp(-jy/ky));
    
    Fz += (sigma * cos(i) * d_theta + tau_x * sin(i) * d_theta);
    Fx += (tau_x * cos(i) * d_theta - sigma * sin(i) * d_theta);
    
    H = w_rad * (cos(i) - cos(theta_f));
    Rb = calc_Rb(H);
    
    Fu += (tau_y * d_theta);
    Fs += (Rb*(w_rad - H*cos(i))*d_theta);
      
    Tz += tau_x * d_theta;
  }
  
  Fx *= (w_rad * w_b);
  Fu *= (w_rad * w_b);
  Fz *= (w_rad * w_b);
  Tz *= (w_rad * w_rad * w_b);
  
  fe[0] = Fx - Fs*sin(beta)*cos(beta);
  fe[1] = Fu + Fs*sin(beta)*sin(beta);

  fe[2] = Fz - Dz*vz;
  fe[3] = Tz;
  fe[4] = fe[1] * w_rad*sin(theta_m);

  Fz = 0.0;
  Fx = 0.0;
  Fu = 0.0;
  Fs = 0.0;
  Tz = 0.0;

}



void tCalc_Fe_negative(double s, double beta, double theta_f, double theta_r,  double vz, double PHI, double *fe)
{
  double i, K, Fx, Fu, Fs, Fz, H, Rb;
  double theta_m, d_theta, Tx, Ty, Tz;
  double sigma, tau_x, tau_y, tau_max, jx, jy;
  double kx,ky;
  double theta_t,Kv;

  Fz = 0.0;
  Fx = 0.0;
  Fu = 0.0;
  Fs = 0.0;
  
  Tx = 0.0;
  Ty = 0.0;
  Tz = 0.0;

  sigma = 0.0;
  tau_x = 0.0;
  tau_y = 0.0;

  jx = 0.0;
  jy  = 0.0;

  H  = 0.0;
  Rb = 0.0;  

  fe[0] = 0.0;
  fe[1] = 0.0;
  fe[2] = 0.0;
  fe[3] = 0.0;
  fe[4] = 0.0;
   
  // adapt theta into ground coordinate
  theta_f -= PHI;
  theta_r -= PHI;
  d_theta = deg2rad(0.25);
  
  // the angle of max stress
  theta_m = (a_0 + a_1 * s) * theta_f;
 
  K = (k_c / w_b + k_phi) * pow(w_rad,n);
  
  beta = fabs(beta);
  
  kx = 0.05;
  ky = 0.03;
  
  theta_t = calc_theta_t(s);
  Kv = calc_Kv(s, theta_f, theta_t);

  for(i=theta_t; i<=theta_f; i+=d_theta)
    {
      sigma = K * pow( (cos(i) - cos(theta_f)), n);
      
      tau_max = (c + sigma*tan(phi));

      jx = calc_jxf(s, theta_f, i, Kv);
      jy = calc_jy(s, theta_f, i, beta);
      
      tau_x = tau_max * (1 - exp(-jx/kx));
      tau_y = tau_max * (1 - exp(-jy/ky));
      
      Fz += (sigma * cos(i) * d_theta + tau_x * sin(i) * d_theta);
      Fx += (tau_x * cos(i) * d_theta - sigma * sin(i) * d_theta);
      
      H = w_rad * (cos(i) - cos(theta_f));
      Rb = calc_Rb(H);
      
      Fu += (tau_y * d_theta);
      Fs += (Rb*(w_rad - H*cos(i))*d_theta);
      
      Tz += (tau_x * d_theta);
    }
  for(i=theta_r; i<theta_t; i+=d_theta)
    {
      sigma = K * pow(cos(theta_f - (i-theta_r)/(theta_m-theta_r)*(theta_f-theta_m)) - cos(theta_f), n);
      
      tau_max = (c + sigma*tan(phi));

      jx = calc_jx(s, theta_t, i);
      jy = calc_jy(s, theta_f, i, beta);
      
      tau_x = tau_max * (1 - exp(-jx/kx));
      tau_y = tau_max * (1 - exp(-jy/ky));
      
      Fz += (sigma * cos(i) * d_theta + tau_x * sin(i) * d_theta);
      Fx += (tau_x * cos(i) * d_theta - sigma * sin(i) * d_theta);
            
      H = w_rad * (cos(i) - cos(theta_f));
      Rb = calc_Rb(H);
      
      Fu += (tau_y * d_theta);
      Fs += (Rb*(w_rad - H*cos(i))*d_theta);
      
      Tz += tau_x * d_theta;
    }
  
  Fx *= (w_rad * w_b);
  Fu *= (w_rad * w_b);
  Fz *= (w_rad * w_b);
  Tz *= (w_rad * w_rad * w_b);
  
  fe[0] = Fx - Fs*sin(beta)*cos(beta);
  fe[1] = Fu + Fs*sin(beta)*sin(beta);

  fe[2] = Fz - Dz*vz;
  fe[3] = Tz;
  fe[4] = fe[1] * w_rad*sin(theta_m);

  Fz = 0.0;
  Fx = 0.0;
  Fu = 0.0;
  Fs = 0.0;
  Tz = 0.0;

}
