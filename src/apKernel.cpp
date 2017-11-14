/*
 * apKernel.cpp
 *
 *  Created on: Dec 4, 2012
 *      Author: agpetit
 */

#include "apKernel.h"

apKernel::apKernel() {
	// TODO Auto-generated constructor stub
	dimension = 1;
	bandwidth = 1;

}

apKernel::~apKernel() {
	// TODO Auto-generated destructor stub
}

apKernel::apKernel(int dim, int k_type) {
dimension = dim;
kernel_type = k_type;
bandwidth.resize(dim,dim);
bandwidth[0][0] = 255*255;
bandwidth[1][1] = 255*255;
bandwidth[2][2] = 255*255;
bandwidth[3][3] = 512*512;
bandwidth[4][4] = 512*512;
sqrtBandwidth.resize(dim,dim);
choleskyDecomposition(bandwidth, sqrtBandwidth, dim);
sqrtBandwidthInv=sqrtBandwidth.inverseByLU();
detB = sqrtBandwidthInv.det();
bwidth.resize(5);
bwidth[0] = 255*255;
bwidth[1] = 255*255;
bwidth[2] = 255*255;
bwidth[3] = 6*512*512;
bwidth[4] = 6*512*512;
}

double
apKernel::KernelDensity(vpColVector &error)
 {

   double density = 0;
   for (int k=0;k<3;k++)
	   density += error[k]*error[k]/bwidth[k];
   if(density < 1)
   {
     //return 1/(2*c)*(dimension+2)*(1-XtX);
	   density =  0.75*(1-density);}
   else
	   density = 0;
   return density;

 }


//Epanechnikov_kernel for an d dimensional Euclidian space R^d
double
apKernel::KernelDensity_EPANECHNIKOV(vpColVector &X)
{

   double XtX= X*X;
   if(XtX < 1)
   {
     //return 1/(2*c)*(dimension+2)*(1-XtX);
	   return 0.75*(1-XtX);}
   else
     return 0;

 }

void apKernel::choleskyDecomposition(vpMatrix &A, vpMatrix &L,int n)
{
    //int n = A.getRows();
    double sum1 = 0.0;
    double sum2 = 0.0;
    double sum3 = 0.0;
    //vector<vector<double> > l(n, vector<double> (n));
    L[0][0] = sqrt(A[0][0]);
    for (int j = 1; j <= n-1; j++)
    L[j][0] = A[j][0]/L[0][0];
    for (int i = 1; i <= (n-2); i++)
    {
    for (int k = 0; k <= (i-1); k++)
    sum1 += pow(L[i][k], 2);
    L[i][i]= sqrt(A[i][i]-sum1);
    for (int j = (i+1); j <= (n-1); j++)
    {
    for (int k = 0; k <= (i-1); k++)
    sum2 += L[j][k]*L[i][k];
    L[j][i]= (A[j][i]-sum2)/L[i][i];
    }
    }
    for (int k = 0; k <= (n-2); k++)
    sum3 += pow(L[n-1][k], 2);
    L[n-1][n-1] = sqrt(A[n-1][n-1]-sum3);
}
