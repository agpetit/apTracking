/*
 * apKernel.h
 *
 *  Created on: Dec 4, 2012
 *      Author: agpetit
 */

#ifndef APKERNEL_H_
#define APKERNEL_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpImagePoint.h>
#include <visp/vpScale.h>
#include <visp/vpMatrix.h>

class apKernel
{
public:

	vpMatrix bandwidth;
	vpMatrix sqrtBandwidth;
	vpMatrix sqrtBandwidthInv;
	unsigned int dimension;
	int kernel_type;
	double detB;
	vpColVector bwidth;

	apKernel();
	apKernel(int dim, int k_type);
	virtual ~apKernel();

	double KernelDensity(vpColVector &error);
	double KernelDensity_EPANECHNIKOV(vpColVector &X);
	void choleskyDecomposition(vpMatrix &A, vpMatrix &L,int n);


};

#endif /* APKERNEL_H_ */
