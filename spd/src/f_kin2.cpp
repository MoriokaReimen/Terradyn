//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : f_kin2( Vector3 pos0, Matrix3 R0 MODEl m)
//
//            compute Forward kinematics
//
//            Input  : position & rotation of Link0 - link_pos
//
//
//            Output : MODEL.link_pos, link_R, link_q
//                           joint_pos, joint_axis
//                           end_pos, end_R
//
// y.sato [2004.12]
// g.ishigami [2005.10]
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#include<iostream>
using namespace std;

#include <ode/ode.h>

#include "../matrix/matrix.hpp"
#include "../matrix/vector.hpp"
#include "../include/spd.h"
#include "../include/rot.h"


void f_kin2( Vector3 pos0, Matrix3 R0, MODEL &m )
{
    int i, j;
    Matrix3 tmp1,tmp2;
    Vector3 tmp3;


    // set position & rotaion of the base(link[0])
    for( i=0 ; i<3 ; i++ ) m.link_pos[0][i] = pos0[i];
    for( i=0 ; i<3*3 ; i++ ) m.link_R[0][i] = R0[i];


    // set a global rotation of base(link[0])
    R2rpy( m.link_R[0], m.link_pos[0]+3 );
    R2q( m.link_R[0], m.link_q[0]);

    if( m.LINKNUM != 1 ) {
        // set link[i] rotation, joint[i] position and joint_axis[i]
        for( i=1 ; i<m.LINKNUM ; i++ ) {
            matrix_mult( 3, 3, 3, m.link_R[m.JA[i][0]], m.link_rR[i], tmp1 );

            if( m.J_type[i] == 0 ) tmp3[2] = m.joint_param[i];
            if( m.J_type[i] == 1 ) tmp3[2] = 0.0;

            rpy2R( tmp3, tmp2 );

            matrix_mult( 3, 3, 3, tmp1, tmp2, m.link_R[i] );

            R2q( m.link_R[i], m.link_q[i]);
            R2rpy( m.link_R[i], m.link_pos[i]+3 );

            for( j=0 ; j<3 ; j++ )
                m.joint_axis[i][j] = m.link_R[i][2+3*j];

        }


        // set joint & link position
        for( i=1 ; i<m.LINKNUM ; i++ ) {
            if( m.J_type[i] == 0 )
                for ( j=0 ; j<3 ; j++ ) {
                    m.joint_pos[i][j] = m.link_pos[m.JA[i][0]][j]
                                        + m.link_R[m.JA[i][0]][3*j]  *m.toJ[i][0]
                                        + m.link_R[m.JA[i][0]][3*j+1]*m.toJ[i][1]
                                        + m.link_R[m.JA[i][0]][3*j+2]*m.toJ[i][2];

                    m.link_pos[i][j] = m.joint_pos[i][j]
                                       + m.link_R[m.JA[i][1]][3*j]  *m.Jto[i][0]
                                       + m.link_R[m.JA[i][1]][3*j+1]*m.Jto[i][1]
                                       + m.link_R[m.JA[i][1]][3*j+2]*m.Jto[i][2];
                }
            if( m.J_type[i] == 1 ) {
                for ( j=0 ; j<3 ; j++ ) {
                    m.joint_pos[i][j] = m.link_pos[m.JA[i][0]][j]
                                        + m.link_R[m.JA[i][0]][3*j]  *m.toJ[i][0]
                                        + m.link_R[m.JA[i][0]][3*j+1]*m.toJ[i][1]
                                        + m.link_R[m.JA[i][0]][3*j+2]*m.toJ[i][2]
                                        + m.link_R[m.JA[i][1]][3*j+2]*m.joint_param[i]/2;

                    m.link_pos[i][j] = m.joint_pos[i][j]
                                       + m.link_R[m.JA[i][1]][3*j]  *m.Jto[i][0]
                                       + m.link_R[m.JA[i][1]][3*j+1]*m.Jto[i][1]
                                       + m.link_R[m.JA[i][1]][3*j+2]*( m.Jto[i][2] + m.joint_param[i]/2 );
                }
            }
        }


        // set end-effector position
        for( i=1 ; i<m.LINKNUM ; i++ ) {
            if( m.EE[i] != 0 ) {
                matrix_mult( 3, 3, 3, m.link_R[i], m.end_rR[m.EE[i]], m.end_R[m.EE[i]] );

                R2rpy( m.end_R[m.EE[i]], m.end_pos[m.EE[i]]+3 );

                for( j=0 ; j<3 ; j++ )
                    m.end_pos[m.EE[i]][j] = m.link_pos[i][j]
                                            + m.link_R[i][3*j]  *m.toE[m.EE[i]][0]
                                            + m.link_R[i][3*j+1]*m.toE[m.EE[i]][1]
                                            + m.link_R[i][3*j+2]*m.toE[m.EE[i]][2];
            }
        }

    }
}
