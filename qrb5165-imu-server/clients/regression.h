
/**
 * The functions in this file are courtesy of Manas Sharma and bragitoff.com
 * Thank you for publishing these functions!
 *
 * TODO: add some fault checking and integrate to librc_math
 */


#include <stdio.h>
#include <math.h>

/**
 * Function that performs Gauss-Elimination and returns the Upper triangular matrix
 * and solution of equations:
 *
 * Pass the augmented matrix (a) as the parameter, and calculate and store the
 * upperTriangular(Gauss-Eliminated Matrix) in it.
**/
static void gaussEliminationLS(int m, int n, double a[m][n], double x[n-1]){
	int i,j,k;
	for(i=0;i<m-1;i++){
		//Partial Pivoting
		for(k=i+1;k<m;k++){
			//If diagonal element(absolute vallue) is smaller than any of the terms below it
			if(fabs(a[i][i])<fabs(a[k][i])){
				//Swap the rows
				for(j=0;j<n;j++){
					double temp;
					temp=a[i][j];
					a[i][j]=a[k][j];
					a[k][j]=temp;
				}
			}
		}
		//Begin Gauss Elimination
		for(k=i+1;k<m;k++){
			double  term=a[k][i]/ a[i][i];
			for(j=0;j<n;j++){
				a[k][j]=a[k][j]-term*a[i][j];
			}
		}

	}
	//Begin Back-substitution
	for(i=m-1;i>=0;i--){
		x[i]=a[i][n-1];
		for(j=i+1;j<n-1;j++){
			x[i]=x[i]-a[i][j]*x[j];
		}
		x[i]=x[i]/a[i][i];
	}
	return;
}




static int _polynomial_regression(int N, int order, double* x, double* y, double* coeff)
{
	int i,j;

	// an array of size 2*n+1 for storing N, Sig xi, Sig xi^2, ...., etc. which are the independent components of the normal matrix
	double X[2*order+1];
	for(i=0;i<=2*order;i++){
		X[i]=0;
		for(j=0;j<N;j++){
			X[i]=X[i]+pow(x[j],i);
		}
	}
	//the normal augmented matrix
	double B[order+1][order+2];
	// rhs
	double Y[order+1];
	for(i=0;i<=order;i++){
		Y[i]=0;
		for(j=0;j<N;j++){
			Y[i]=Y[i]+pow(x[j],i)*y[j];
		}
	}
	for(i=0;i<=order;i++){
		for(j=0;j<=order;j++){
			B[i][j]=X[i+j];
		}
	}
	for(i=0;i<=order;i++){
		B[i][order+1]=Y[i];
	}

	gaussEliminationLS(order+1,order+2,B,coeff);

	// printf("The polynomial fit is given by the equation:\n");


	// for(i=0;i<=order;i++){
	// 	printf("%lfx^%d+",coeff[i],i);
	// }

	return 0;
}