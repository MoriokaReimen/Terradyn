//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : model_ode_result3( MODEL,
//                              Vector3, Matrix3 )
//            Get result of ODE dynamics computation
//
// y.sato [2004.12]
// g.ishigami [2005.10]
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#include <iostream>
using namespace std;

#include <ode/ode.h>

#include "../matrix/matrix.h"
#include "../matrix/vector.h"
#include "../include/model.h"
#include "../include/rot.h"


void model_ode_result3( MODEL &m, Vector3 pos0, Matrix3 R0, double dt)
{
    int i, j;
    const dReal *tmp;


    tmp = dBodyGetPosition( m.link[0] );
    for( i=0 ; i<3 ; i++ )
        pos0[i] = tmp[i];


    tmp = dBodyGetRotation( m.link[0]);
    for( i=0 ; i<3 ; i++ )
        for( j=0 ; j<3 ; j++ )
            R0[3*i+j] = tmp[4*i+j];


    if( m.LINKNUM != 1 ) {
        for( i=1 ; i<m.LINKNUM ; i++ ) {
            if( m.J_type[i] == 0 ) {
                m.joint_param[i] = m.joint_param_init[i] + dJointGetHingeAngle( m.joint[i] );
                m.joint_param_vel[i] = dJointGetHingeAngleRate( m.joint[i] );
            }
            if( m.J_type[i] == 1 ) {
                m.joint_param[i] = dJointGetSliderPosition( m.joint[i] );
                m.joint_param_vel[i] = dJointGetSliderPositionRate( m.joint[i] );
            }
        }
    }

    // get velocities of each link , add by genya.
    for(i=0; i<m.LINKNUM; i++) {
        tmp = dBodyGetLinearVel(m.link[i]);
        for(j=0; j<3; j++)
            m.link_vv[i][j] = tmp[j];

        tmp = dBodyGetAngularVel(m.link[i]);
        for(j=0; j<3; j++)
            m.link_ww[i][j] = tmp[j];
    }

    // calculate acceleration of each link and each joint. add by genya.

    for(i=0; i<m.LINKNUM; i++) {
        for(j=0; j<3; j++) {
            m.link_vd[i][j] = (m.link_vv[i][j] - m.link_pre_vv[i][j]) / (dt*0.001);
            m.link_wd[i][j] = (m.link_ww[i][j] - m.link_pre_ww[i][j]) / (dt*0.001);

            m.link_pre_vv[i][j] = m.link_vv[i][j];
            m.link_pre_ww[i][j] = m.link_ww[i][j];
        }

        m.joint_param_acc[i] = (m.joint_param_vel[i] - m.joint_param_pre_vel[i]) / (dt*0.001);
        m.joint_param_pre_vel[i] = m.joint_param_vel[i];
    }

}
