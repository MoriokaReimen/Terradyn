//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Terra mehcanics + Dynamics = Terra dyn
// Header file
// Toyoura sand definition file            
// g.ishigami [2007.11]
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#ifndef TERRADYN_H
#define TERRADYN_H

#define M_PI 3.14159265358979323846

#define w_rad  0.055
#define w_b    0.064

/*#define K_flag 0
#define k_c   0.0
#define k_phi 3.48334*1000*1000*10
#define n  1.703
*/
#define K_flag 1
#define k_c   0.0
#define k_phi 1203.54
#define n  1.703


#define a_0  0.4
#define a_1  0.15
#define c  0.0

#define rho  1.4905*1000.0
#define phi  38.0 * M_PI / 180.0

#define kappa 0.5

#define delta 10.0 * M_PI / 180.0  // roll
#define alpha 0.0 * M_PI / 180.0  // pitch

double tInit_sinkage(double);
void   tCalc_Fe_positive(double, double, double, double, double, double, double*);
void   tCalc_Fe_negative(double, double, double, double, double, double, double*);
	
#endif
