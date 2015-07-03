
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : model_init( dWorldID, MODEL )
//            Initialze MODEL parameters in dWorldID
//
// y.sato [2004.12]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#include <iostream>
using namespace std;

#include <ode/ode.h>

#include "../matrix/matrix.h"
#include "../matrix/vector.h"
#include "../include/spd.h"
#include "../include/rot.h"



void model_init( dWorldID &w, MODEL &m )
{

    int i,j;
    Matrix3 tmp1,tmp2;
    Vector3 tmp3;


    // set a global rotation of base(link[0])
    rpy2R( m.link_pos[0]+3, m.link_R[0] );
    R2q( m.link_R[0], m.link_q[0]);


    if( m.LINKNUM != 1 ) {
        // set relative rotation of each bodies
        for( i=1 ; i<m.LINKNUM ; i++ )
            rpy2R( m.link_pos[i]+3, m.link_rR[i] );


        // calculate global rotation of each links
        // !!!!!!!!!! CAUTION !!!!!!!!!
        for ( i=1 ; i<m.LINKNUM ; i++ ) {
            matrix_mult( 3, 3, 3, m.link_R[m.JA[i][0]], m.link_rR[i], tmp1 );

            if( m.J_type[i] == 0 ) tmp3[2] = m.joint_param_init[i];
            if( m.J_type[i] == 1 ) tmp3[2] = 0.0;

            rpy2R( tmp3, tmp2 );

            matrix_mult( 3, 3, 3, tmp1, tmp2, m.link_R[i] );

            R2q( m.link_R[i], m.link_q[i]);
            R2rpy( m.link_R[i], m.link_pos[i]+3 );

            // cout << "link " << i << endl;
// 	  for( j=0 ; j<3 ; j++ )
// 	    cout << ' ' << m.link_R[i][3*j] << ' ' << m.link_R[i][3*j+1] << ' ' << m.link_R[i][3*j+2] << endl;
        }


        // set joint & link position
        for( i=1 ; i<m.LINKNUM ; i++ )
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

        // set end-effector position
        for( i=1 ; i<m.LINKNUM ; i++ ) {
            if( m.EE[i] != 0 ) {
                rpy2R( m.end_pos[m.EE[i]]+3, m.end_rR[m.EE[i]] );

                matrix_mult( 3, 3, 3, m.link_R[i], m.end_rR[m.EE[i]], m.end_R[m.EE[i]] );

                R2rpy( m.end_R[m.EE[i]], m.end_pos[m.EE[i]]+3 );
                R2q( m.end_R[m.EE[i]], m.end_q[m.EE[i]] );

                for( j=0 ; j<3 ; j++ )
                    m.end_pos[m.EE[i]][j] = m.link_pos[i][j]
                                            + m.link_R[i][3*j]  *m.toE[m.EE[i]][0]
                                            + m.link_R[i][3*j+1]*m.toE[m.EE[i]][1]
                                            + m.link_R[i][3*j+2]*m.toE[m.EE[i]][2];
            }
        }


        // Set_Z_Axis_As_a_Rotation_Axis_of_Each_Joints
        dVector3 rela_axis;
        vector_z( 3, rela_axis );
        rela_axis[2] = -1;


        // setting rotaton axis of each joints
        for( i=1 ; i<m.LINKNUM ; i++ )
            for ( j=0 ; j<3 ; j++ )
                m.joint_axis[i][j] = m.link_R[i][3*j]*rela_axis[0]
                                     + m.link_R[i][3*j+1]*rela_axis[1] + m.link_R[i][3*j+2]*rela_axis[2];

    }


    // create link
    for( i=0 ; i<m.LINKNUM ; i++ ) {
        dMassSetParameters( &m.mass[i], m.link_M[i],
                            0, 0, 0, //dReal cgx, dReal cgy, dReal cgz,
                            m.link_I_fixed[i][0*3+0], m.link_I_fixed[i][1*3+1], m.link_I_fixed[i][2*3+2], //I11, I22, I33,
                            m.link_I_fixed[i][0*3+1], m.link_I_fixed[i][0*3+2], m.link_I_fixed[i][1*3+2]);//I12, I13, I23);

        m.link[i] = dBodyCreate ( w );
        dBodySetMass ( m.link[i], &m.mass[i] );
        dBodySetPosition ( m.link[i], m.link_pos[i][0], m.link_pos[i][1], m.link_pos[i][2] );
        dBodySetQuaternion ( m.link[i], (m.link_q[i]) );
        dBodySetGravityMode( m.link[i], 1 );
    }


    if( m.LINKNUM != 1 ) {
        // create joints
        for ( i=1 ; i<m.LINKNUM ; i++ ) {
            if( m.J_type[i] == 0 ) {
                m.joint[i] = dJointCreateHinge( w, 0 );
                dJointAttach( m.joint[i], m.link[m.JA[i][0]], m.link[m.JA[i][1]] );
                dJointSetHingeAnchor ( m.joint[i], m.joint_pos[i][0], m.joint_pos[i][1], m.joint_pos[i][2] );
                dJointSetHingeAxis ( m.joint[i], m.joint_axis[i][0], m.joint_axis[i][1], m.joint_axis[i][2] );
                dJointSetHingeParam ( m.joint[i], dParamFMax, 0.0 );

                // lower limit of a joint ratation
                //dJointSetHingeParam (joint[i],dParamLoStop,-M_PI/2);
                // higher limit of a joint ratation
                //dJointSetHingeParam (joint[i],dParamHiStop,M_PI/2);

                // calc joint_param
                m.joint_param[i] = m.joint_param_init[i] + dJointGetHingeAngle( m.joint[i] );
            }
            if( m.J_type[i] == 1 ) {
                m.joint[i] = dJointCreateSlider( w, 0 );
                dJointAttach( m.joint[i], m.link[m.JA[i][0]], m.link[m.JA[i][1]] );
                dJointSetSliderAxis( m.joint[i], m.joint_axis[i][0], m.joint_axis[i][1], m.joint_axis[i][2] );
                dJointSetSliderParam ( m.joint[i], dParamFMax, 0.0 );

                // calc joint_param
                m.joint_param[i] = dJointGetSliderPosition( m.joint[i] );
            }
        }


    }

    // zeros of previous velocity add by genya
    for(i=0; i<m.LINKNUM; i++) {
        for(j=0; j<3; j++) {
            m.link_pre_vv[i][j] = 0.0;
            m.link_pre_ww[i][j] = 0.0;
        }
        m.joint_param_pre_vel[i] = 0.0;
    }

    cout << endl << "Model init OK " << endl << endl;

}
