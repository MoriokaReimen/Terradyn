//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : calc_HH_q( MODEL, ans )
//            compute inertia matrix HH_q
//
//            ans = (LINKNUM-1)x(LINKNUM-1) matrix
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


void calc_HH_q( MODEL &m, double *ans )
{
    
    int i;
    Matrix3 tmp1, tmp2, tmp3;
    double *tmp4, *tmp5, *tmp6, *tmp7;
    
    
    // if not single link
    if( m.LINKNUM != 1 )
    {
	matrix_Z( m.LINKNUM-1, m.LINKNUM-1, ans );
	
	tmp4 = matrix_get( 3, m.LINKNUM-1 );
	tmp5 = matrix_get( 3, m.LINKNUM-1 );
	tmp6 = matrix_get( m.LINKNUM-1, m.LINKNUM-1 );
	tmp7 = matrix_get( m.LINKNUM-1, m.LINKNUM-1 );
	
	for( i=1 ; i<m.LINKNUM ; i++ )
	{
	    
	    // calc trans(JJ_r)
	    matrix_trans( 3, m.LINKNUM-1, m.JJ_r[i], tmp4 );
	    
	    
	    // calc Ii
	    matrix_mult( 3, 3, 3, m.link_R[i], m.link_I_fixed[i], tmp1 );
	    matrix_trans( 3, 3, m.link_R[i], tmp2 );
	    matrix_mult( 3, 3, 3, tmp1, tmp2, tmp3 );
	    
	    
	    // calc trans(JJ_r)*Ii
	    matrix_mult( m.LINKNUM-1, 3, 3, tmp4, tmp3, tmp5 );
	    
	    
	    // calc trans(JJ_r)*Ii*JJ_r
	    matrix_mult( m.LINKNUM-1, 3, m.LINKNUM-1, tmp5, m.JJ_r[i], tmp6 );
	    
	    
	    // calc trans(JJ_t)
	    matrix_trans( 3, m.LINKNUM-1, m.JJ_t[i], tmp4 ); 
	    
	    
	    // calc trans(JJ_ti)*JJ_ti
	    matrix_mult( m.LINKNUM-1, 3, m.LINKNUM-1, tmp4, m.JJ_t[i], tmp7 );
	    
	    
	    // calc M*trans(JJ_ti)*JJ_ti
	    matrix_scale( m.LINKNUM-1, m.LINKNUM-1, m.link_M[i], tmp7, tmp7 );
	    
	    
	    // calc HH_q
	    matrix_add( m.LINKNUM-1, m.LINKNUM-1, tmp6, tmp7, tmp7 );
	    matrix_add( m.LINKNUM-1, m.LINKNUM-1, tmp7, ans, ans );
	    
	}
	
	free( tmp4 );
	free( tmp5 );
	free( tmp6 );
	free( tmp7 );
	
    }
    
}
