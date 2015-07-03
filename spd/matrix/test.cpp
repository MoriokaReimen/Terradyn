#include <stdio.h>

#include "matrix.h"
#include "vector.h"

int main()
{
    int i,m,n;
    
    m = 3;
    n = 2;
    
    double *a,*a2,*b,*c;
    double *v,*v2;
    double *l,*u;
    
    
    a = get_matrix( m );
    l = get_matrix( m );
    u = get_matrix( m );
    
    
    for(i=0;i<m*m;i++)
	a[i] = i;

    a[0] = 100;
    a[1] = 30;
    a[3] = 1;
    a[4] = 1;
    a[5] = 12;
    
    print_matrix( m, a );
    
    LU( m, a, l, u );
    double d;
    d = det( m, a );
    printf("%lf\n", d );
    print_matrix( m, l );
    print_matrix( m, u );
    
}
