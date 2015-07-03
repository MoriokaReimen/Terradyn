//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : calc_jr( MODEL, ans )
//            Rotational Jacobians w.r.t. link centroid
//
//            ans[i][3x(LINKNUM-1)]
//             = 3x(LINKNUM-1) jacobian matrix of Link[i]
//               stored in 3xi, the others are zero
//
// y.sato [2004.12]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#include<iostream>
using namespace std;

#include "../matrix/matrix.h"
#include "../matrix/vector.h"
#include "../include/spd.h"
#include "../include/rot.h"


void calc_jr( MODEL &m, double **ans )
{

    int i, j, n0, n1;
    Vector3 tmp1,tmp2;


    // if not single link
    if( m.LINKNUM != 1 ) {
        for( i=1 ; i<m.LINKNUM ; i++ ) {
            vector_z( (m.LINKNUM-1)*3, ans[i] );

            n1 = m.JA[i][1];
            n0 = m.JA[i][0];

            for( j=0 ; j<m.LINKNUM ; j++ ) {
                if( m.J_type[n1] == 0 ) { // rotational joint
                    vector_cpy( 3, m.joint_axis[n1], tmp2 );
                } else if( m.J_type[n1] == 1 ) { // prismatic joint
                    vector_z( 3, tmp1 );
                    vector_cpy( 3, tmp1, tmp2 );
                }

                ans[i][n1-1]                 = tmp2[0];
                ans[i][n1-1+(m.LINKNUM-1)]   = tmp2[1];
                ans[i][n1-1+(m.LINKNUM-1)*2] = tmp2[2];

                if(n0==0) break;
                n1 = m.JA[n0][1];
                n0 = m.JA[n0][0];

            }

        }
    }
}
