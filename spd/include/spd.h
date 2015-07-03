#include <iostream>
using namespace std;
#include <string>
#include "../include/model.h"
#include "../include/define.h"
#include "../include/common.h"

void model_param( string, MODEL & );
void model_init( dWorldID &, MODEL & );
void model_draw( MODEL & );
void model_ode_result(MODEL &m, Vector3, Matrix3, double*, double* );
void model_ode_result2(MODEL &m, Vector3, Matrix3 );
void model_ode_result3(MODEL &m, Vector3, Matrix3, double );
void f_kin( Vector3, Matrix3, double *, MODEL & );
void f_kin2( Vector3, Matrix3, MODEL & );
void calc_jt( MODEL &, double ** );
void calc_jr( MODEL &, double ** );
void calc_jte( MODEL &, int, int, int[], double* );
void calc_jre( MODEL &, int, int, int[], double* );
void calc_HH_q( MODEL &, double * );
void calc_AA( MODEL &, Matrix3, double *, double ** );
void calc_AA2( MODEL &, Matrix3, double ** );
