/*
 * apDetection.h
 *
 *  Created on: Nov 14, 2012
 *      Author: agpetit
 */

#ifndef APDETECTION_H_
#define APDETECTION_H_

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
//#include "apViews.h"

class apDetection
{
public :
	typedef enum
	{
	ORIENTED_CHAMFER,
	SHAPE_CONTEXT,
	STEGER
	}similarityMeasure;
public:

    //apViews::similarityMeasure similarityMs;
    int nbParticles;
    int nr;
    int nw;
    int nscales;
    int sx;
    int sy;
    int stheta;
    int nbimax;
    int startingLevel;
    int sample;
    double cannyTh1;
    double cannyTh2;
    similarityMeasure similarityMs;
    double lambda;
    double mud;
    double lambdao;
    double sigmaf;

	apDetection();
	apDetection(const apDetection &detect);
	virtual ~apDetection();
	const apDetection& operator=(const apDetection &detect);
	void print( ) ;
};

#endif /* APDETECTION_H_ */
