#include <iostream>
using namespace std;
#include <ode/ode.h>
#include "../include/define.h"
#include "../include/common.h"

class MODEL{
    
 public:
    
    int LINKNUM;
    int **JA;
    int *EE;
    int *J_type;
    
    // link
    dBodyID *link;
    Vector6 *link_pos;
    Matrix3 *link_R;
    Matrix3 *link_rR;
    Vector4 *link_q;

    Vector3 *link_vv;
    Vector3 *link_pre_vv;
    Vector3 *link_vd;

    Vector3 *link_ww;
    Vector3 *link_pre_ww;
    Vector3 *link_wd;

    // link mass
    dMass *mass;
    dReal *link_M;
    Matrix3 *link_I;
    Matrix3 *link_I_fixed;
    
    // joint
    dJointID *joint;
    Vector6 *joint_pos;
    Vector3 *toJ;
    Vector3 *Jto;
    Vector3 *joint_axis;
    dReal *joint_param;
    dReal *joint_param_init;
    dReal *joint_param_vel;
    dReal *joint_param_pre_vel;
    dReal *joint_param_acc;

    Vector6 *end_pos;
    Matrix3 *end_R;
    Matrix3 *end_rR;
    Vector3 *toE;
    Vector4 *end_q;
    
    
    double **JJ_t;
    double **JJ_r;
    double **AA;
    Matrix3 HH_w;
    double *HH_wq;
    double *HH_q;
    
    //_/_/_/     constructor of arrays for ODE      _/_/_//
    void constructor(){
	
	int i;
	
	JA = new int*[LINKNUM];
	for( i=0 ; i<LINKNUM ; i++ ) JA[i] = new int[2];
	
	EE = new int[LINKNUM];
	J_type = new int[LINKNUM];
	
	
	// link
	link     = new dBodyID[LINKNUM];
	link_pos = new Vector6[LINKNUM];
	link_R   = new Matrix3[LINKNUM];
	link_rR  = new Matrix3[LINKNUM];
	link_q   = new Vector4[LINKNUM];

	link_vv = new Vector3[LINKNUM];
	link_pre_vv = new Vector3[LINKNUM];
	link_vd = new Vector3[LINKNUM];
	
	link_ww = new Vector3[LINKNUM];
	link_pre_ww = new Vector3[LINKNUM];
	link_wd = new Vector3[LINKNUM];

	// link mass
	mass         = new dMass[LINKNUM];
	link_M       = new dReal[LINKNUM];	
	link_I       = new Matrix3[LINKNUM];
	link_I_fixed = new Matrix3[LINKNUM];
	
	// joint
	joint            = new dJointID[LINKNUM];
	joint_pos        = new Vector6[LINKNUM];	
	toJ              = new Vector3[LINKNUM];
	Jto              = new Vector3[LINKNUM];
	joint_axis       = new Vector3[LINKNUM];	
	joint_param      = new dReal[LINKNUM]; 
	joint_param_init = new dReal[LINKNUM]; 
	joint_param_vel  = new dReal[LINKNUM];
	joint_param_pre_vel  = new dReal[LINKNUM];
	joint_param_acc  = new dReal[LINKNUM];
	
	JJ_t = new double*[LINKNUM];
	JJ_r = new double*[LINKNUM];
	AA = new double*[LINKNUM];	
	for( i=0 ; i<LINKNUM ; i++ )
	{
	    JJ_t[i] = new double[(LINKNUM-1)*3];
	    JJ_r[i] = new double[(LINKNUM-1)*3];    
	    AA[i] = new double[3*3];
	}
	
	HH_wq = new double[(LINKNUM-1)*3];
	HH_q = new double[(LINKNUM-1)*(LINKNUM-1)];
	
    }
    
    void constructor2( int e_num ){
	
	// end-effector
	end_pos = new Vector6[e_num+1];
	end_R   = new Matrix3[e_num+1];
	end_rR  = new Matrix3[e_num+1];
	toE     = new Vector3[e_num+1];
	end_q   = new Vector4[e_num+1];
    }
    
};
