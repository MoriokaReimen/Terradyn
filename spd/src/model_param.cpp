//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : model_param( string, MODEL )
//            Read Model Parameters from model file
//
// y.sato [2004.12]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#include <iostream>
using namespace std;
#include <fstream>
#include <strings.h>

#include "../include/spd.h"
#include "../include/rot.h"

#define _LINESKIP ifs >> str;


void model_param( string filename, MODEL &m )
{
    int i,j;
    int E_NUM = 0;
    int check;
    double joint_param_tmp;
    string str;
    
    ifstream ifs(filename.c_str());
    if(!ifs)
    {
	cerr << "Cannot open " << filename << endl;
	exit(1);
    }
    
    _LINESKIP; //###Parameters_for_MODEL
    
    ifs >> str >> m.LINKNUM;
    cout << "link_number = " << m.LINKNUM << endl;
    
    /* create arrays of MODEL class */
    m.constructor();
    
    
    _LINESKIP; //###Joint_Attach	
    if( m.LINKNUM != 1 )
	for( i=1 ; i<m.LINKNUM ; i++ )
	    ifs >> str >> m.JA[i][0] >> m.JA[i][1]  >> str;
    
    
    _LINESKIP; //###Joint_Type_[_0=rotational__1=prismatic]
    if( m.LINKNUM != 1 )
    {
	ifs >> str;
	for( i=1 ; i<m.LINKNUM ; i++ )
	    ifs >> m.J_type[i];
	ifs >> str;
    }
    
    _LINESKIP; //###EE
    ifs >> str;
    for( i=0 ; i<m.LINKNUM ; i++ )
    {
	ifs >> m.EE[i];
	if( m.EE[i] > E_NUM ) E_NUM = m.EE[i];
    }
    ifs >> str;
    m.constructor2( E_NUM );
    
    
    
    _LINESKIP; //###Set_Link0(base)
    ifs >> str
	>> m.link_pos[0][0] >> m.link_pos[0][1] >> m.link_pos[0][2]
	>> m.link_pos[0][3] >> m.link_pos[0][4] >> m.link_pos[0][5]
	>> str;
    
    for( i=0 ; i<3 ; i++ ) m.link_pos[0][i+3] = deg2pi(m.link_pos[0][i+3]);
    
    
    _LINESKIP; //###Relative_Coordinate_System:Roll_Pitch_Yaw_Angle_of_Each_Bodies{x-y-z}
    if( m.LINKNUM != 1 )
    {
	for( i=1 ; i<m.LINKNUM ; i++ )
	{
	    ifs >> str >> m.link_pos[i][3] >> m.link_pos[i][4] >> m.link_pos[i][5] >> str;
	    
	    m.link_pos[i][3] = deg2pi(m.link_pos[i][3]);
	    m.link_pos[i][4] = deg2pi(m.link_pos[i][4]);
	    m.link_pos[i][5] = deg2pi(m.link_pos[i][5]);
	    
	    cout << "rpy"<< i << " = " <<  m.link_pos[i][3] << " " << m.link_pos[i][4] << " " << m.link_pos[i][5] << endl;
	}
    }
    
    
    _LINESKIP; //###Initial_Joint_Parameter
    if( m.LINKNUM != 1 )
   {
	ifs >> str;
	for( i=1 ; i<m.LINKNUM ; i++ )
	{
	    ifs >> joint_param_tmp;
	    
	    if( m.J_type[i] == 0 ) m.joint_param_init[i] = deg2pi(joint_param_tmp);
	    if( m.J_type[i] == 1 ) m.joint_param_init[i] = joint_param_tmp;
	    
	    cout << "joint_param_init[" << i << "] = " << m.joint_param_init[i] << endl;
	}
	ifs >> str;
    }
    
    
    _LINESKIP; //###Joint_Position_Vector
    if( m.LINKNUM != 1 )
    {
	for( i=1 ; i<m.LINKNUM ; i++ )
	{
	    ifs >> str >> m.toJ[i][0] >> m.toJ[i][1] >> m.toJ[i][2] >> str;
	    ifs >> str >> m.Jto[i][0] >> m.Jto[i][1] >> m.Jto[i][2] >> str;
	}
    }
    
    _LINESKIP; //###End-Effector_position
    if( E_NUM != 0 )
	for( i=1 ; i<E_NUM+1 ; i++ )
	    ifs >> str >> m.toE[i][0] >> m.toE[i][1] >> m.toE[i][2] >> str;
    
    
    _LINESKIP; //###End-Effector_rotation
    if( E_NUM != 0 )
	for( i=1 ; i<E_NUM+1 ; i++ )
	    ifs >> str >> m.end_pos[i][3] >> m.end_pos[i][4] >> m.end_pos[i][5] >> str;
    
    
    
    _LINESKIP; //###Mass_Parameter
    ifs >> str;
    for( i=0 ; i<m.LINKNUM ; i++ )
	ifs >> m.link_M[i];
    ifs >> str;
    
    _LINESKIP; //###Intertia_Matrix
    _LINESKIP; //###[_I11_I12_I13_]
    _LINESKIP; //###[_I12_I22_I23_]
    _LINESKIP; //###[_I13_I23_I33_]
    for( i=0 ; i<m.LINKNUM ; i++ )
    {
	ifs >> str;
	ifs >> str >> m.link_I_fixed[i][0*3+0];
	ifs >> str >> m.link_I_fixed[i][1*3+1];
	ifs >> str >> m.link_I_fixed[i][2*3+2];
	ifs >> str >> m.link_I_fixed[i][0*3+1]; m.link_I_fixed[i][1*3+0] = m.link_I_fixed[i][0*3+1];
	ifs >> str >> m.link_I_fixed[i][0*3+2]; m.link_I_fixed[i][2*3+0] = m.link_I_fixed[i][0*3+2];
	ifs >> str >> m.link_I_fixed[i][1*3+2]; m.link_I_fixed[i][2*3+1] = m.link_I_fixed[i][1*3+2];
    }
    
    /*for( i=0 ; i<m.LINKNUM ; i++ )
      {
	cout << "Inertia Matrix Link " << i << endl;
	for( j=0 ; j<3 ; j++ )
	  cout << m.link_I_fixed[i][j*3+0]
	       << ' '
	       << m.link_I_fixed[i][j*3+1]
	       << ' '
	       << m.link_I_fixed[i][j*3+2] << endl;
      }*/
    
    //###EOF
    ifs >> str >> check;
    if( check != 777 )
      {
	cerr << "error : model parameter" << endl;
	exit(0);
    }
    
    cout << endl << "read model paramter finish" << endl << endl;
}
