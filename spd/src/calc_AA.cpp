//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : calc_AA( MODEL, Matrix3, j_param, ans )
//       compute coordinate transformation matrices AA
//
//            *ans = [3*3]
//
// G.ishigami [2005.10]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#include<iostream>
using namespace std;

#include "../matrix/matrix.h"
#include "../matrix/vector.h"
#include "../include/spd.h"
#include "../include/rot.h"


void calc_AA( MODEL &m, Matrix3 R0, double *j_param, double **ans )
{

    int i,BB;
    Vector3 tmp1;
    Matrix3 tmp2;

    matrix_cpy(3, 3, R0, ans[0]);			// base AA[0] is equivalent to A0

    for(i = 1; i < m.LINKNUM; i++) {
        BB = m.JA[i][0];
        R2rpy(m.link_rR[i], tmp1);

        if(m.J_type[i] == 0)
            tmp1[2] += j_param[i];

        rpy2R(tmp1,tmp2);

        if(BB == 0)
            matrix_mult(3, 3, 3, R0, tmp2, ans[i]);

        else
            matrix_mult(3, 3, 3, ans[BB], tmp2, ans[i]);

    }

}
