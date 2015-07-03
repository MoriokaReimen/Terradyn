
/*/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : main(),		  simloop()
//			  Main Function   Simulation Loop
//
// g.ishigami [2007.11]
//
//		   Rover Simulation
//		   Model = el_dorado
//		   
// modified in 2007.11
//  - confirm control gains at the steers, drives and rocker
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_*/

#include<iostream> 
#include<fstream>
#include<math.h> 
using namespace std;

#include "spd/include/define.h"
#include "spd/include/spd.h"
#include "spd/include/rot.h"
#include "spd/matrix/matrix.h"
#include "spd/matrix/vector.h"

#ifdef DRAWSTUFF
//#include <drawstuff/drawstuff.h>
#include "drawstuff/drawstuff.h"
#include "spd/include/draw.h"
#endif

#include "terra/terradyn2.h"

// Rover roll angle /////////////////////////////////////
#define roll 0.0*M_PI/180.0  //phi=0
//#define roll 5.0*M_PI/180.0  //phi=5
//#define roll 10.0*M_PI/180.0  //phi=10
//#define roll 15.0*M_PI/180.0  //phi=15
//#define roll 20.0*M_PI/180.0  //phi=20

//micro
//#define roll 0.0*M_PI/180.0  //phi=0
//#define roll 1.0*M_PI/180.0  //phi=1
//#define roll 2.0*M_PI/180.0  //phi=2
//#define roll 3.0*M_PI/180.0  //phi=3
//#define roll 4.0*M_PI/180.0  //phi=4
//#define roll 5.0*M_PI/180.0  //phi=5
//#define roll 6.0*M_PI/180.0  //phi=6
//#define roll 7.0*M_PI/180.0  //phi=7
//#define roll 8.0*M_PI/180.0  //phi=8
//#define roll 9.0*M_PI/180.0  //phi=9
//#define roll 10.0*M_PI/180.0  //phi=10

#define wheel_num 4
#define rad2deg(x) x/M_PI*180.0  // Radian to Degree
#define deg2rad(x) x*M_PI/180.0  // Degree to Radian

static double interval = 1.0; //[msec]
static dWorldID world;
static MODEL rover;
/////////////////////////////////////////////////////////////////
///
///   parameter
///
//////////////////////////////////////////////////////////////////
//static dReal wheelbase = 0.222; 	// wheel_base/2
//static dReal tred      = 0.150; 	// tred base/2 
//static dReal H	       = 0.1865; 	// ground-clearance of the centorid
static dReal wheelbase = 0.300; 	// wheel_base/2
static dReal tred      = 0.275; 	// tred base/2 
//static dReal H	       = 0.21153; 	// ground-clearance of the centorid   ‘S‘Ì‚ÌdS
//static dReal H	       = 0.21116;  //when phi=5
//static dReal H	       = 0.1994;  //when phi=10
//static dReal H	       = 0.18708;  //when phi=15
//static dReal H	       = 0.15977;  //when phi=20
//static dReal H	       = 0.17435;

//micro
static dReal H	       = 0.21153;  //phi=00
//static dReal H	       = 0.20908;  //phi=01
//static dReal H	       = 0.20664;  //phi=02
//static dReal H	       = 0.20418;  //phi=03
//static dReal H	       = 0.20173;  //phi=04
//static dReal H	       = 0.19927;  //phi=05
//static dReal H	       = 0.19680;  //phi=06
//static dReal H	       = 0.19432;  //phi=07
//static dReal H	       = 0.19183;  //phi=08
//static dReal H	       = 0.18933;  //phi=09
//static dReal H	       = 0.18682;  //phi=10
//static dReal H	       = 0.17435;  //phi=15
//static dReal H	       = 0.15977;  //phi=20

//New parameter
static dReal Lu        = 0.27304;//phi15‚Ì‚Æ‚«
static dReal Ld        = 0.28704;
//static dReal d_h       = 0.14737;  //‹r‚ÌL‚Ñ48.12
//static dReal d_h       = 0.04812;  //‹r‚ÌL‚Ñ phi05

static dReal d_h       = 0.0;  //phi=00
//static dReal d_h       = 0.0097;  //phi=01
//static dReal d_h       = 0.0194;  //phi=02
//static dReal d_h       = 0.0291;  //phi=03
//static dReal d_h       = 0.0389;  //phi=04
//static dReal d_h       = 0.0486;  //phi=05
//static dReal d_h       = 0.0584;  //phi=06
//static dReal d_h       = 0.0683;  //phi=07
//static dReal d_h       = 0.0781;  //phi=08
//static dReal d_h       = 0.0881;  //phi=09
//static dReal d_h       = 0.0980;  //phi=10
//static dReal d_h       = 0.1490;  //phi=15
//static dReal d_h       = 0.2024;  //phi=20

dReal h[wheel_num], d_steer[wheel_num], d_wheel_vel[wheel_num];
dReal theta_f[wheel_num],theta_r[wheel_num];
dReal slip[wheel_num], beta[wheel_num], wheel_v[3*wheel_num];
dReal PHI[wheel_num];

Vector3 vel_tmp;
Vector3 base_pos0, base_Q0, base_v0, base_w0, base_vd0, base_wd0; 

int flag[wheel_num];

dReal W_all;
Vector3 pos0;
Matrix3 R0;

Matrix3 AA_slope;

dReal vel_sum[wheel_num];

int sim_time= 0;

/*ofstream outfile1("revised_steer30_data_2.txt");
ofstream outfile2("revised_steer30_data_2_fe.txt");
ofstream outfile3("revised_steer30_data_2_pos.txt");
//*/

/*ofstream outfile1("steer15_01_data.txt");
ofstream outfile2("steer15_01_data_fe.txt");
ofstream outfile3("steer15_01_data_pos_I.txt");
ofstream outfile4("steer15_01_data_pos.txt");
//*/
ofstream outfile1("data.txt");
ofstream outfile2("data_fe.txt");
ofstream outfile3("data_pos_I.txt");
ofstream outfile4("data_pos.txt");
ofstream outfile5("data_pos_link0.txt");  //new
ofstream outfile6("phi.txt");
ofstream outfile7("link0_rotation_init.txt");
ofstream outfile8("get_rotation.txt");
ofstream outfile9("get_position.txt");
ofstream outfile10("data_slip.txt");
ofstream outfile11("data_beta.txt");
ofstream outfile12("data_h.txt");

#ifdef DRAWSTUFF
void simLoop(int pause);
#else
void simLoop();
#endif

int sgn(double a)
{
  if(a > 0)
    return 1;
  else if(a < 0)
    return -1;
  else
    return 0;
}

int main(void)
{
  int i,j;

  Vector3 tmp;
  Vector3 slope_angle;
  //Matrix3 R0;
  Matrix3 inv_AA;
  dReal theta_fc;
  dReal W_tmp, h_c, delta_plus, alpha_plus;

  // zeros
  vector_z(3, tmp);
  vector_z(wheel_num, h);
  vector_z(wheel_num, theta_f);
  vector_z(wheel_num, theta_r);
  vector_z(wheel_num, slip);
  vector_z(wheel_num, beta);
  vector_z(wheel_num*3, wheel_v);
  vector_z(wheel_num, vel_sum);

  vector_z(3, vel_tmp);
  matrix_Z(3,3,inv_AA);
	
  vector_z(3, slope_angle);
  matrix_Z(3,3,AA_slope);

  W_tmp = 0.0;
  W_all = 0.0;
  theta_fc = 0.0;
  delta_plus = 0.0;
  alpha_plus = 0.0;
  h_c = 0.0;

  // --------- model reading ----------------------------

  world = dWorldCreate();
  dWorldSetGravity( world, 0.0, 0.0, -9.8);
  //model_param( "spd/model/el_dorado2_phi00", rover );
  //model_param( "spd/model/el_dorado2_phi05", rover );
  //model_param( "spd/model/el_dorado2_phi10", rover );
  //model_param( "spd/model/el_dorado2_phi15", rover );
  //model_param( "spd/model/el_dorado2_phi20", rover );

  //model_param( "spd/model/el_dorado2_phi00_center.txt", rover );
  //model_param( "spd/model/el_dorado2_phi05_center.txt", rover );
  //model_param( "spd/model/el_dorado2_phi10_center.txt", rover );
  //model_param( "spd/model/el_dorado2_phi15_center.txt", rover );
  //model_param( "spd/model/el_dorado2_phi20_center.txt", rover );

  //model_param( "spd/model/dora2_phi00_center_intertia.txt", rover );
  //model_param( "spd/model/dora2_phi05_center_intertia.txt", rover );

  model_param( "spd/model/dora2_micro_phi00_modified.txt", rover);
  //model_param( "spd/model/dora2_micro_phi05_modified.txt", rover);
  //model_param( "spd/model/dora2_micro_phi10_modified.txt", rover);
  //model_param( "spd/model/dora2_micro_phi15_modified.txt", rover);
  //model_param( "spd/model/dora2_micro_phi20_modified.txt", rover);
  //model_param( "spd/model/dora2_micro_phi15_modified_center.txt", rover);

  //model_param( "spd/model/dora2_micro_phi00.txt", rover);
  //model_param( "spd/model/dora2_micro_phi01.txt", rover);
  //model_param( "spd/model/dora2_micro_phi02.txt", rover);
  //model_param( "spd/model/dora2_micro_phi03.txt", rover);
  //model_param( "spd/model/dora2_micro_phi04.txt", rover);
  //model_param( "spd/model/dora2_micro_phi05.txt", rover);
  //model_param( "spd/model/dora2_micro_phi06.txt", rover);
  //model_param( "spd/model/dora2_micro_phi07.txt", rover);
  //model_param( "spd/model/dora2_micro_phi08.txt", rover);
  //model_param( "spd/model/dora2_micro_phi09.txt", rover);
  //model_param( "spd/model/dora2_micro_phi10.txt", rover);

  //---------- simulation start -------------------------

  // mass	
  for(i=0; i<rover.LINKNUM; i++)
	W_all += rover.link_M[i];

  W_all *= 9.8;
	
  // --- calculation of static sinkage ------------------
  for(i=0; i<wheel_num; i++){//original
    switch(i){
    case 0:
      W_tmp = (W_all/4.0) * (1.0-tan(delta)*H/tred)*(1.0-tan(alpha)*H/wheelbase);
      break;
    case 1:
      W_tmp = (W_all/4.0) * (1.0-tan(delta)*H/tred)*(1.0+tan(alpha)*H/wheelbase);
      break;
    case 2:
      W_tmp = (W_all/4.0) * (1.0+tan(delta)*H/tred)*(1.0+tan(alpha)*H/wheelbase);
      break;
    case 3:
      W_tmp = (W_all/4.0) * (1.0+tan(delta)*H/tred)*(1.0-tan(alpha)*H/wheelbase);
      break;
    }
  /*for(i=0; i<wheel_num; i++){
    switch(i){
    case 0:
      W_tmp = (W_all/2.0) * (Ld*cos(delta-roll)-d_h*sin(delta-roll)-H*sin(delta-roll))/(Ld*cos(delta-roll)+Lu*cos(delta-roll)-d_h*sin(delta-roll))*(1.0-tan(alpha)*H/wheelbase);
      break;
    case 1:
      W_tmp = (W_all/2.0) * (Ld*cos(delta-roll)-d_h*sin(delta-roll)-H*sin(delta-roll))/(Ld*cos(delta-roll)+Lu*cos(delta-roll)-d_h*sin(delta-roll))*(1.0+tan(alpha)*H/wheelbase);
      break;
    case 2:
      W_tmp = (W_all/2.0) * (Lu*cos(delta-roll)+H*sin(delta-roll))/(Ld*cos(delta-roll)+Lu*cos(delta-roll)-d_h*sin(delta-roll))*(1.0+tan(alpha)*H/wheelbase);
      break;
    case 3:
      W_tmp = (W_all/2.0) * (Lu*cos(delta-roll)+H*sin(delta-roll))/(Ld*cos(delta-roll)+Lu*cos(delta-roll)-d_h*sin(delta-roll))*(1.0-tan(alpha)*H/wheelbase);
      break;
    }*/
    
    // --- static sinkage & static contact angle -----
    
    theta_fc = tInit_sinkage2(W_tmp, roll);
    h[i] = w_rad * (1-cos(theta_fc - alpha)) / cos(alpha);
	//h[i] = w_rad * (1-cos(theta_fc - alpha)) / cos(alpha-roll);

    theta_f[i] = theta_fc;
    theta_r[i] = 2*alpha - theta_fc;
    
    W_tmp = 0.0;
  }
  cout << "W="<<W_all << "\t sinkage=" << h[0] <<endl;

  // --- renew initial position of the base ---
	
  h_c = (h[0] + h[1] + h[2] + h[3])/4.0;
  cout << "W="<<W_all << "\t h_c =" << h_c <<endl;

  delta_plus = atan2( (h[3]-h[0]), 2.0*tred );
  alpha_plus = atan2( (h[1]-h[0]), 2.0*wheelbase);
	
  rover.link_pos[0][2] -= h_c;		
  //rover.link_pos[0][3] += (delta + delta_plus);
  rover.link_pos[0][3] += (delta + delta_plus - roll);
  rover.link_pos[0][4] += (-alpha-alpha_plus);
  rover.link_pos[0][5] = deg2rad(0.0);

  outfile7 << rad2deg(rover.link_pos[0][3]) << '\t'
		<< rad2deg(rover.link_pos[0][4]) << '\t' << rad2deg(rover.link_pos[0][5]) << endl;

  // --- 
  // move base to set (0,0,h') at the slope coordinate
  // ---
  
  slope_angle[0] = delta;
  slope_angle[1] = alpha;
  slope_angle[2] = 0.0;
  rpy2R(slope_angle, AA_slope);  
  
  tmp[0] = rover.link_pos[0][0];
  tmp[1] = rover.link_pos[0][1];
  tmp[2] = rover.link_pos[0][2];
  
  matrix_inv(3, AA_slope, inv_AA);
  matrix_mult(3, 3, 1, inv_AA, tmp, base_pos0);
  
  base_pos0[1] -= base_pos0[1]; // parallel shift in y_slope axis
  vector_z(3,tmp);
  
  matrix_mult(3, 3, 1, AA_slope, base_pos0,tmp);
  rover.link_pos[0][0] = tmp[0];
  rover.link_pos[0][1] = tmp[1];
  rover.link_pos[0][2] = tmp[2];
  matrix_Z(3,3,inv_AA); 
  
  // --- steering angles -----------------------
  
  /*d_steer[0] = deg2rad(15.0); 	// front- left
  d_steer[1] = deg2rad(15.0);	// rear - left
  d_steer[2] = deg2rad(15.0);	// rear - right
  d_steer[3] = deg2rad(15.0); 	// front- right
  //*/
  d_steer[0] = deg2rad(15.0); 	// front- left
  d_steer[1] = deg2rad(15.0);	// rear - left
  d_steer[2] = deg2rad(15.0);	// rear - right
  d_steer[3] = deg2rad(15.0); 	// front- right

  // --- wheel angular velocity ----------------
  //  x=100  --> 0.614 [rad/sec] --> 
  //    0.614*0.052(wheel radius) = 0.032 [m/sec] = 3.2 [cm/sec]
  /*
  d_wheel_vel[0] = -0.614*0.5;
  d_wheel_vel[1] = -0.614*0.5;
  d_wheel_vel[2] = -0.614*0.5;
  d_wheel_vel[3] = -0.614*0.5;//*/
  //d_wheel_vel[0] = -0.304;
  //d_wheel_vel[1] = -0.304;
  //d_wheel_vel[2] = -0.310;
  //d_wheel_vel[3] = -0.310;
  //
  d_wheel_vel[0] = -0.176;  // 18[mm/sec]=0.018[m/sec]=0.176*0.102(wheel radius)
  d_wheel_vel[1] = -0.176;
  d_wheel_vel[2] = -0.176;
  d_wheel_vel[3] = -0.176;

  // --- initializations -----------------------

  for(i=0; i<wheel_num; i++){
    slip[i] = 1.0;
    beta[i] = 0.0;
    PHI[i]  = 0.0;
    flag[i] = 1;
    rover.joint_param_init[i+3] = d_steer[i];
  }

  // --- model init, calculation of forward kinematics ----
 
  model_init( world, rover );
  model_ode_result3( rover, pos0, R0, interval);
  f_kin2( pos0, R0, rover ); 

  // --- coor-trans of wheel-linear-velocities -----------

  for(i=0; i<wheel_num; i++){
    matrix_inv(3, rover.link_R[i+3], inv_AA);
    matrix_mult(3, 3, 1, inv_AA, rover.link_vv[i+3], vel_tmp);
    
    for(j=0; j<3; j++)
      wheel_v[3*i+j] = vel_tmp[j];
  }
    
  //  --- base conditions @ base-coordinate system ------
  matrix_inv(3, rover.link_R[0], inv_AA);
  R2rpy(R0, tmp);
  matrix_mult(3, 3, 1, inv_AA, pos0, base_pos0);
  matrix_mult(3, 3, 1, inv_AA, tmp, base_Q0);
  matrix_mult(3, 3, 1, inv_AA, rover.link_vv[0], base_v0);
  matrix_mult(3, 3, 1, inv_AA, rover.link_ww[0], base_w0);
  matrix_mult(3, 3, 1, inv_AA, rover.link_vd[0], base_vd0);
  matrix_mult(3, 3, 1, inv_AA, rover.link_wd[0], base_wd0);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // --- Simulation Loop starts -----	
  //__________________________________

#ifdef DRAWSTUFF	
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = "/usr/local/share/ode-0.5/drawstuff/textures";
	
  // run simulation
  dsSimulationLoop (0,0,640,480,&fn);
  
#else	
  do{
    simLoop();
    
    counter++;
    
  }while( counter < 10000);
#endif
	
  dWorldDestroy( world );
	
  cout << "!!! All Program Finished !!!" << endl;
	
  return 0;  
}

///_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
///_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
///_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
//	Simulation Loop  
//
///_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
///_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
///_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#ifdef DRAWSTUFF
void simLoop(int pause)
#else

void simLoop()
#endif
{

	//new
	//dMatrix3 get_R;
	const dReal *get_R,*get_pos;
	dVector3 get_rpy;
	dMatrix3 get_tmp;
	vector_z(3, get_rpy);
	
  
  int i,j;
  
  Vector3 R0_tmp;
  Matrix3 inv_AA;
  Vector3 fe_tmp0, fe_tmp1;

  dReal Fe[3*wheel_num];
  dReal tau[rover.LINKNUM];
  dReal Kp, Kd, Ki;

  dReal X, X_dush,g_z;
  Vector3 wheel_x, wheel_x_dush; 
  dReal h_v, h_PHI;
  dReal wheel_x_tmp;

  vector_z(3, fe_tmp0);
  vector_z(3, fe_tmp1);
  vector_z(3*wheel_num, Fe);
  vector_z(rover.LINKNUM, tau);
  vector_z(3, R0_tmp);
  matrix_Z(3,3,inv_AA);
  vector_z(3, vel_tmp);

  vector_z(3 ,wheel_x);
  vector_z(3 ,wheel_x_dush);

  X = 0.0;
  X_dush = 0.0;
  g_z = 0.0;
  h_v = 0.0;
  h_PHI = 0.0;
  wheel_x_tmp = 0.0;

  sim_time++;

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  wheel rotation control by PI feedback 
  //________________________________________	

  Kp = 1.0;
  Ki = 0.1;
  for(i=0; i<wheel_num; i++){
    //tau[i+7] = Kp*(d_wheel_vel[i] - rover.joint_param_vel[i+7]) + Kd*(0.0 - rover.joint_param_acc[i+7]);
    vel_sum[i] += (d_wheel_vel[i] - rover.joint_param_vel[i+7]);	
    tau[i+7] = Kp*(d_wheel_vel[i] - rover.joint_param_vel[i+7]) + Ki*vel_sum[i];
  }	

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // steering angle control by PD feedback
  //________________________________________

  //Kp = 2.0;//original
  //Kd = 2.0;
  //Kp = 100.0;
  Kp = 50.0;//‚Ù‚Ú0‚É‚È‚Á‚½D
  Kd = 0.5;
  //Kd = 2.5;
  for(i=0; i<wheel_num; i++)
    tau[i+3] = Kp*(d_steer[i] - rover.joint_param[i+3]) + Kd*(0.0 - rover.joint_param_vel[i+3]);
	
	
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // rocker-link torques generated by wires
  //	  are simulated by PD feedback as follows:
  //_______________________________________________

  //Kp = 1.0;//original
  //Kd = 1.0;
  //Kp = 100.0;
  Kp = 50.0;//yawŠp‚Ù‚Ú0‚É‚È‚Á‚½
  Kd = 0.5;
  /*tau[1] = Kp*( -rover.joint_param[2]-rover.joint_param[1])
    + Kd*(-rover.joint_param_vel[2]-rover.joint_param_vel[1]);
  
  tau[2] = Kp*( -rover.joint_param[1]-rover.joint_param[2]) 
    + Kd*(-rover.joint_param_vel[1]-rover.joint_param_vel[2]);
  //*/
  tau[1] = Kp*( -rover.joint_param[2]-rover.joint_param[1]) + Kd*(-rover.joint_param_vel[1]);
  tau[2] = Kp*( -rover.joint_param[1]-rover.joint_param[2]) + Kd*(-rover.joint_param_vel[2]);
  //tau[1] = Kp*(0.0 - rover.joint_param[1]) + Kd*(0.0 - rover.joint_param_vel[1]);
  //tau[2] = Kp*(0.0 - rover.joint_param[2]) + Kd*(0.0 - rover.joint_param_vel[2]);
 

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Add torques to each joint
  //____________________________________

  for(i=1; i<rover.LINKNUM; i++)
    dJointAddHingeTorque( rover.joint[i], tau[i]);
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Calculation Fe of each wheel based on Terramechanics
  //__________________________________________________________________

  /////////////////////new///////////////////////////////////////////////
  //  dbodygetrotation
  ///////////////////////////////////////////////////////////////////////
  get_R = dBodyGetRotation(rover.link[0]);
  for( i=0 ; i<3 ; i++ )
    for( j=0 ; j<3 ; j++ )
      get_tmp[3*i+j] = get_R[4*i+j];
  R2rpy(get_tmp, get_rpy);

  get_pos = dBodyGetPosition(rover.link[0]);





  for(i=0; i<wheel_num; i++){
    if( flag[i] == 1 ){
      if(slip[i] >= 0.0)
		  tCalc_Fe_positive2(slip[i], beta[i], theta_f[i], theta_r[i], 2500*rover.link_vv[i+3][2], PHI[i], fe_tmp0, fabs(get_rpy[1]));
		  //tCalc_Fe_positive2(slip[i], beta[i], theta_f[i], theta_r[i], 2500*rover.link_vv[i+3][2], PHI[i], fe_tmp0, roll);
      else
		  tCalc_Fe_positive2(slip[i]*0.0, beta[i], theta_f[i], theta_r[i], 2500*rover.link_vv[i+3][2], PHI[i], fe_tmp0, fabs(get_rpy[1]));
		//tCalc_Fe_positive2(slip[i]*0.0, beta[i], theta_f[i], theta_r[i], 2500*rover.link_vv[i+3][2], PHI[i], fe_tmp0, roll);
		//tCalc_Fe_negative(slip[i], beta[i], theta_f[i], theta_r[i], 25*rover.link_vv[i+3][2], PHI[i], fe_tmp0);
      
      if(beta[i] > 0)
	fe_tmp0[1] *= -1.0;
      if(rover.joint_param_vel[i+7] > 0.0)
	fe_tmp0[0] *= -1.0;
      
      //~~~~~~~~~~~~~~~~~~~ CAUTION !!! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // The coordinate system of wheels defined by terramechanics are quivalent to
      // the one of steer parts. Thus, coor-trans should be executed by rover.link_R[i+3].
      //____________________________________________________________________________________
      
      matrix_mult(3, 3, 1, rover.link_R[i+3], fe_tmp0, fe_tmp1);
      
      Fe[3*i+0] = fe_tmp1[0];
      Fe[3*i+1] = fe_tmp1[1];
      Fe[3*i+2] = fe_tmp1[2];
    }
    else{
      Fe[3*i]	= 0.0;
      Fe[3*i+1] = 0.0;
      Fe[3*i+2] = 0.0;
    }
  
    //if(rover.joint_param_vel[i+7] > 0.0)
    //fe_tmp0[3] *= -1.0; 
    //Add Rotation resistance
    //dJointAddHingeTorque(rover.joint[i+7], fe_tmp0[3]); 		
    //dBodyAddRelTorque(rover.link[i+7], 0, 0, fe_tmp0[3]);		
    
    // Add Self-Aligning Torque
    //dJointAddHingeTorque(rover.joint[i+3], sgn(beta[i])*fe_tmp0[4]);			
    
    // Add External forces	
    dBodyAddForce(rover.link[i+7], Fe[3*i], Fe[3*i+1], Fe[3*i+2]);
    //dBodyAddForce(rover.link[i+7], 0.0, 0.0, Fe[3*i+2]);//*/
    //cout << fe_tmp0[0] << '\t' << fe_tmp0[1] << '\t' << fe_tmp0[2] << endl;
    
  }
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Calculation of Dynamics and Kinematics
  //_____________________________________________
       
  dWorldStep( world, (dReal)interval/1000.0 );	
  model_ode_result3( rover, pos0, R0, interval);
  f_kin2( pos0, R0, rover ); 
  model_draw( rover );


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  coor-trans of wheel-linear-velocities
  //___________________________________________
	
  for(i=0; i<wheel_num; i++){
    matrix_inv(3, rover.link_R[i+3], inv_AA);
    matrix_mult(3, 3, 1, inv_AA, rover.link_vv[i+3], vel_tmp);
    
    for(j=0; j<3; j++)
      wheel_v[3*i+j] = vel_tmp[j];	
    
    matrix_Z(3,3,inv_AA);	
    vector_z(3, vel_tmp);
  }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //   base conditions @ slope-coordinate system
  //_________________________________________________
  
  matrix_inv(3, AA_slope, inv_AA);
  matrix_mult(3, 3, 1, inv_AA, pos0, base_pos0);  
  R2rpy(R0, R0_tmp); 
  matrix_mult(3, 3, 1, inv_AA, R0_tmp, base_Q0);
  matrix_mult(3, 3, 1, inv_AA, rover.link_ww[0], base_w0);
  matrix_Z(3,3,inv_AA); //*/
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //   base conditions @ base-coordinate system
  //  
  //   v0 should be expressed by the base coordinate system
  //_________________________________________________
  
  matrix_inv(3, rover.link_R[0], inv_AA);
  matrix_mult(3, 3, 1, inv_AA, rover.link_vv[0], base_v0); 
  //matrix_mult(3, 3, 1, inv_AA, rover.link_ww[0], base_w0);
  matrix_Z(3,3,inv_AA); //*/
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //   calculation of wheel slip-ratio and slip-angle	
  //____________________________________________________

  for(i=0; i<wheel_num; i++){
    if( sgn(wheel_v[3*i+0]) * sgn(rover.joint_param_vel[i+7]*(-1.0)) >= 0){
      if( fabs(wheel_v[3*i+0]) <= w_rad * fabs(rover.joint_param_vel[i+7]) ){
	slip[i] = 1.0 - ( fabs(wheel_v[3*i+0]) / (w_rad*fabs(rover.joint_param_vel[i+7])) );	 
	if(slip[i] > 1.0 ) 
	  slip[i] = 1.0;
      }
      else if( fabs(wheel_v[3*i+0]) > w_rad * fabs(rover.joint_param_vel[i+7]) ){
	slip[i] = ((w_rad*fabs(rover.joint_param_vel[i+7])) / fabs( wheel_v[3*i+0]) ) - 1.0;	 
	if(slip[i] < -1.0 ) 
	  slip[i] = -1.0;
      }
      else;
    }
    else
      slip[i] = 1.0;
    
    beta[i] = atan(wheel_v[3*i+1]/fabs(wheel_v[3*i+0]));
  }
  

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //	Calculation of wheel sinkage
  //__________________________________________
	
  for(i=0; i<wheel_num; i++){
    g_z = tan(delta) * rover.link_pos[i+7][1] + tan(alpha)*rover.link_pos[i+7][0];
    
    for(j=0; j<3; j++)
      wheel_x[j] = rover.link_R[i+3][j];
    
    X = sqrt(wheel_x[0]*wheel_x[0] + wheel_x[1]*wheel_x[1] + wheel_x[2]*wheel_x[2]);
    
    wheel_x_dush[0] = wheel_x[0];
    wheel_x_dush[1] = wheel_x[1];
    wheel_x_dush[2] = 0.0;
    
    X_dush = sqrt(wheel_x_dush[0]*wheel_x_dush[0] + wheel_x_dush[1]*wheel_x_dush[1] 
		  + wheel_x_dush[2]*wheel_x_dush[2]);
    
    wheel_x_tmp = vector_inner(3, wheel_x, wheel_x_dush);
    PHI[i] = acos(wheel_x_tmp / (X * X_dush));
    
    if(wheel_x[2]<0)
      PHI[i] = -PHI[i];
    
    h[i] = w_rad - rover.link_pos[i+7][2] + g_z;
    h_PHI = h[i] + w_rad*( 1/cos(PHI[i]) -1);
    h_v = h_PHI * cos(PHI[i]);

    if(h_v < 0.0){// || h_v > w_rad){
      cout << endl << "wheel" << i << "is floating!!" << endl;
      theta_f[i] = 0.0;
      theta_r[i] = 0.0;
      flag[i] =0;
    }
    else{
      theta_f[i] =  acos(1 - h_v/w_rad) + PHI[i];
      //theta_r[i] = -acos(1 - 1.15*h_v/w_rad) + PHI[i];
      theta_r[i] = -acos(1 - (kappa)*h_v/w_rad) + PHI[i];
      if(slip[i] < 0.0)
	theta_r[i] = -acos(1 - (1.0 + slip[i])*h_v/w_rad) + PHI[i];
      
      flag[i] = 1;
    }
    
  }//*/
	
  if(sim_time%100==0){	
    outfile1 << sim_time*interval << '\t'
	     << slip[0] << '\t' << slip[1]<< '\t' 
	     << slip[2] << '\t' << slip[3]<< '\t'
	     << rad2deg(beta[0]) << '\t' << rad2deg(beta[1]) << '\t'
	     << rad2deg(beta[2]) << '\t' << rad2deg(beta[3]) << '\t'
	     << rad2deg(rover.joint_param[1]) << '\t' << rad2deg(rover.joint_param[2]) << '\t'
	     << rad2deg(rover.joint_param[3]) << '\t' << rad2deg(rover.joint_param[4]) << '\t'
	     << rad2deg(rover.joint_param[5]) << '\t' << rad2deg(rover.joint_param[6]) << '\t'
	     << rover.joint_param_vel[7] << '\t' << rover.joint_param_vel[8] << '\t'
	     << rover.joint_param_vel[9] << '\t' << rover.joint_param_vel[10] << '\t'
          // << rad2deg(rover.joint_param[1]) << '\t' <<rad2deg(rover.joint_param[2]) << '\t'
	     << h[0] << '\t'<< h[1] << '\t'<< h[2] << '\t'<< h[3] << '\t'
	     << endl; 
    
    /*outfile4 << sim_time*interval << '\t'
	     << tau[1] << '\t' << tau[2] << '\t'
	     << tau[3] << '\t' << tau[4] << '\t' << tau[5] << '\t' << tau[6] << '\t'
	     << tau[7] << '\t' << tau[8] << '\t' << tau[9] << '\t' << tau[10] << endl;//*/

    outfile4 << sim_time*interval << '\t'
	     << base_pos0[0] << '\t' << base_pos0[1] <<'\t' 
	     << base_v0[0] << '\t' << base_v0[1] <<'\t' 
	     << rad2deg(base_Q0[2]) << endl;
             //<< V_b << '\t' << rad2deg(BETA) << endl;
    
    outfile2 << sim_time*interval << '\t'
	     << Fe[0] << '\t' << Fe[3] << '\t' << Fe[6] << '\t' << Fe[9]	<< '\t' 
	     << Fe[1] << '\t' << Fe[4] << '\t' << Fe[7] << '\t' << Fe[10] << '\t' 
	     << Fe[2] << '\t' << Fe[5] << '\t' << Fe[8] << '\t' << Fe[11] << endl;	
    
    outfile3 << sim_time*interval << '\t'
	     << pos0[0] << '\t' << pos0[1] <<'\t' << pos0[2] << '\t'
	     << rad2deg(rover.link_pos[0][3]) <<'\t'
	     << rad2deg(rover.link_pos[0][4]) <<'\t'
	     << rad2deg(rover.link_pos[0][5]) << endl;

	//new
	outfile5 << sim_time*interval << '\t'
	     << rover.link_pos[0][0] << '\t' << rover.link_pos[0][1] <<'\t' 
	     << rover.link_pos[0][2] << endl;
    
	outfile6 << sim_time*interval << '\t'
	     << rad2deg(rover.link_pos[0][3]) << '\t' << rad2deg(rover.link_pos[0][4]) <<'\t' 
	     << rad2deg(rover.link_pos[0][5]) << '\t' 
		 << rad2deg(rover.joint_param[3]) << '\t'
		 << rad2deg(rover.joint_param[4]) << '\t' << rad2deg(rover.joint_param[5]) <<'\t' 
	     << rad2deg(rover.joint_param[6]) << '\t' <<  endl;

		/*outfile6 << sim_time*interval << '\t'
	     << rad2deg(rover.link_pos[0][3]) << '\t' << rad2deg(rover.link_pos[0][4]) <<'\t' 
	     << rad2deg(rover.link_pos[0][5]) << '\t' << rad2deg(rover.joint_param[6]) << '\t'
		 << h[0] << '\t' << h[1] <<'\t' 
	     << h[2] << '\t' << h[3]<<  endl;*/
	
/*
		outfile6 << sim_time*interval << '\t'
	     << rad2deg(PHI[0]) << '\t' << rad2deg(PHI[1]) <<'\t' 
	     << rad2deg(PHI[2]) << '\t' << rad2deg(PHI[3]) << '\t'
		 << h[0] << '\t' << h[1] <<'\t' 
	     << h[2] << '\t' << h[3]<<  endl;
	*/	//outfile6 << sim_time*interval << '\t'
	 //    << wheel_v[0] << '\t' << wheel_v[1] <<'\t' 
	 //    << wheel_v[2] << '\t'
		// << wheel_v[3] << '\t' << wheel_v[4] <<'\t' 
	 //    << wheel_v[5] << '\t'
		// << wheel_v[6] << '\t' << wheel_v[7] <<'\t' 
	 //    << wheel_v[8] << '\t'
		// << wheel_v[9] << '\t' << wheel_v[10] <<'\t' 
	 //    << wheel_v[11] <<endl;

	outfile8 << sim_time*interval << '\t'
	     << rad2deg(get_rpy[0]) <<'\t'
	     << rad2deg(get_rpy[1]) <<'\t'
	     << rad2deg(get_rpy[2]) << '\t' << endl;

	outfile9 << sim_time*interval << '\t'
	     << get_pos[0] <<'\t'
	     << get_pos[1] <<'\t'
	     << get_pos[2] << '\t' 
		 << (get_pos[1]) / cos(roll) << '\t' << endl;

	outfile10 << sim_time*interval << '\t'
	     << slip[0] << '\t' << slip[1]<< '\t' 
	     << slip[2] << '\t' << slip[3]<< '\t' << endl;

	outfile11 << sim_time*interval << '\t'
	     << rad2deg(beta[0]) << '\t' << rad2deg(beta[1]) << '\t'
	     << rad2deg(beta[2]) << '\t' << rad2deg(beta[3]) << '\t' << endl;

	outfile12 << sim_time*interval << '\t'
	     << h[0] << '\t' << h[1] << '\t'
		 << h[2] << '\t' << h[3] << '\t' << endl;

    cout << sim_time*interval << '\t' << base_pos0[0] << '\t'<< base_pos0[1] << '\t'
		<< rad2deg(rover.link_pos[0][3]) << '\t' << rad2deg(rover.link_pos[0][5]) << endl;
	/*cout << sim_time*interval << '\t' << base_pos0[0] << '\t'<< base_pos0[1] << '\t'
		<< slip[0] << '\t' << rad2deg(rover.link_pos[0][3]) << endl;*/  
  }//*/
}
