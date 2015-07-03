//_/_/_/_/ vector library _/_/_/_//

#include <stdio.h>
#include <stdlib.h>
#include "vector.h"

//_/_/_/ get vector /_/_/_//
double *vector_get( int n )
{
    return ( double * ) malloc( sizeof( double ) * n );
}

//_/_/_/ copy vector /_/_/_//
void vector_cpy( int n, double *a, double *u )
{
    int i;
    
    for ( i = 0 ; i < n ; i++ )
    {
	u[ i ] = a[ i ];
    }
}

//_/_/ print vector _/_//
void vector_print( int n, double *a )
{
    int	i;
    
    for ( i=0 ; i<n ; i++ )
	printf("%f ", a[ i ]);  
    printf("\n");
}


//_/_/ print vector _/_//
void vector_print_int( int n, int *a )
{
    int	i;
    
    for ( i=0 ; i<n ; i++ )
	printf("%d ", a[ i ]);  
    printf("\n");
}


//_/_/_/ addition /_/_/_//
void vector_add( int n, double *a, double *b, double *c )
{
    int i;
    
    for( i=0 ; i<n ; i++ )
    {
	c[i] = a[i] + b[i];
    }
}


//_/_/_/ subtraction /_/_/_//
void vector_sub( int n, double *a, double *b, double *c )
{
    int i;
    
    for( i=0 ; i<n ; i++ )
    {
	c[i] = a[i] - b[i];
    }
}


//_/_/_/ return an inner product of matrix a & b /_/_/_//
double vector_inner( int n, double *a, double *b )
{
    int i;
    double sum;
    double *tmp = vector_get( n );
    
    sum = 0.0;
    
    for( i=0 ; i<n ; i++ )
    {
	tmp[i] = a[i]*b[i];
	sum +=tmp[i];
    }
  
    free( tmp );
    
    return sum;
}


//_/_/_/ return a cross product of matrix a(3x1) & b(3x1) /_/_/_//
void vector_cross3( double *a, double *b, double *c )
{
    c[0] = a[1]*b[2]-a[2]*b[1];
    c[1] = a[2]*b[0]-a[0]*b[2];
    c[2] = a[0]*b[1]-a[1]*b[0];
}


//_/_/_/ return a tilde matrix form vector3 /_/_/_//
void vector_tilde3( double *a, double *b )
{
    b[0] = 0;
    b[1] = -a[2];
    b[2] = a[1];
    b[3] = a[2];
    b[4] = 0;
    b[5] = -a[0];
    b[6] = -a[1];
    b[7] = a[0];
    b[8] = 0;
}

void vector_z( int n, double *a )
{
    int i;
    
    for( i=0 ; i<n ; i++ )
	a[i] = 0.0;
}

void vector_i( int n, double *a )
{
    int i;
    
    for( i=0 ; i<n ; i++ )
	a[i] = 1.0;
}

void vector_scale( int n, double s, double *a, double *b )
{
    int i;
    
    double *A = vector_get(n);
    
    vector_cpy(n,a,A);
    
    for( i=0 ; i<n ; i++ )
	b[i] = s*A[i];
    
    free(A);
}
