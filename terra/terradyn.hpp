//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Terra mehcanics + Dynamics = Terra dyn
// Header file
// Toyoura sand definition file
// g.ishigami [2007.11] ---> Yujiro [2012.11]
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#ifndef TERRADYN_H
#define TERRADYN_H


// Wheel parameters /////////////////////////////////////
#define w_rad  0.102	// Wheel radius [m]
#define w_b 0.100	// Wheel width [m]
/////////////////////////////////////////////////////////

// Soil parameters(Toyoura Std. Sand) ///////////////////
#define SOIL_K 0.028329167		// Shear deformation modulus [m]
#define SOIL_C 0.0			// Cohesion [Pa]
#define SOIL_PHI 38.0*M_PI/180.0	// Internal friction angle [rad]
#define SOIL_N 1.703			// Pressure-sinkage modulus [-]
#define SOIL_KC 0.0			// Pressure-sinkage modulus [-]
#define SOIL_KPHI 1203.54/9.80665	// Pressure-sinkage modulus [-]
#define SOIL_A0 0.40			// Maximum stress angle mudulus [-]
#define SOIL_A1 0.15			// Maximum stress angle mudulus [-]
#define SOIL_RHO 1.49*1000		// Bulk density [kg/m^3]
#define SOIL_GAMMA SOIL_RHO*9.80665 	// Unit weight [N/m^3]
//#define WHEEL_KAPPA 0.5		// Exit angle coefficient [-]

// Soil-wall parameters
#define WALL_DELTA 0.0*M_PI/180.0	// External friction angle [rad]
#define WALL_CA 0.0			// Adhesion [Pa]
///////////////////////////////////////////////////////

// Terrain parameters /////////////////////////////////
//#define delta 0.0 * M_PI/180.0 // roll [rad]
#define delta 10.0 * M_PI/180.0 // roll [rad]
//#define delta 15.0 * M_PI/180.0 // roll [rad]
//#define delta 20.0 * M_PI/180.0 // roll [rad]
#define alpha 0.0 * M_PI/180.0  // pitch [rad]
///////////////////////////////////////////////////////

// Simulation parameters //////////////////////////////
//#define DELTA_THETA 0.0001	// É¢É∆ [rad] for integration  (too late)
//#define DELTA_Y 0.01		// É¢y [m] for integration
#define DELTA_THETA 0.01	// É¢É∆ [rad] for integration
#define DELTA_Y 0.01		// É¢y [m] for integration
///////////////////////////////////////////////////////

// Global variables ///////////////////////////////////
//static double omega_d;		// Soil slip angle (uphill sidewall) [rad]
//static double omega_u;		// Soil slip angle (downhill sidewall) [rad]
static double kappa;		// Exit angle coefficient [-]
static double w_b_eff;		// Effective wheel width [m]
///////////////////////////////////////////////////////

double tInit_sinkage2(double, double);
void   tCalc_Fe_positive2(double, double, double, double, double, double, double*, double);
void   tCalc_Fe_negative2(double, double, double, double, double, double, double*, double);

#endif
