/*
 * apDetection.h
 *
 *  Created on: Nov 14, 2012
 *      Author: agpetit
 */

#ifndef APLEARN_H_
#define APLEARN_H_

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include "apDetection.h"

class VISP_EXPORT apLearn
{
public:

	apDetection::similarityMeasure similarityMS;
    int sampleViewsRho;
    int sampleViewsTheta;
    int sampleViewsPhi;
    int sampleRViewsTheta;
    int sampleRViewsPhi;
    int nOverlap;
    double dist;

	apLearn();
	apLearn(const apLearn &learn);
	virtual ~apLearn();
	const apLearn& operator=(const apLearn &learn);
	void print( ) ;
};

#endif /* APLEARN_H_ */
