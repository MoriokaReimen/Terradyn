//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : calc_jre()
//
//
//
// hiro [2005.01]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#include<iostream>
#include<stdio.h>
using namespace std;

#include "../matrix/matrix.h"
#include "../matrix/vector.h"
#include "../include/model.h"
#include "../include/rot.h"


void calc_jre( MODEL &m, int end_pos_n, int ctrl_joint_n, int ctrl_joint[], double *ans )
{

    int i;
    Vector3 tmp1,tmp2;

    // if not single link
    if( m.LINKNUM != 1 ) {
        vector_z( m.LINKNUM*3, ans );

        for( i=0 ; i<ctrl_joint_n ; i++ ) {
            if( m.J_type[ctrl_joint[i]] == 0 ) { // rotational joint
                vector_cpy( 3, m.joint_axis[ctrl_joint[i]], tmp2 );
            } else if( m.J_type[ctrl_joint[i]] == 1 ) { // prismatic joint
                vector_z( 3, tmp1 );
                vector_cpy( 3, tmp1, tmp2 );
            }

            ans[ctrl_joint[i]-1]                 = tmp2[0];
            ans[ctrl_joint[i]-1+(m.LINKNUM-1)]   = tmp2[1];
            ans[ctrl_joint[i]-1+(m.LINKNUM-1)*2] = tmp2[2];
        }
    }
}
