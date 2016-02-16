/*
 * apLearn.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: agpetit
 */
#include <stdlib.h>
#include "apLearn.h"

apLearn::apLearn() {
	// TODO Auto-generated constructor stub
    //similarityMs = BACKGROUND_GREY;
    sampleViewsRho = 15;
    sampleViewsTheta = 15;
    sampleViewsPhi = 15;
    sampleRViewsTheta = 5;
    sampleRViewsPhi = 5;
    nOverlap = 1;
    dist = 3;

}
apLearn::apLearn(const apLearn &learn)
{
  *this = learn;
}

const
apLearn& apLearn::operator=(const apLearn &learn)
{
    //similarityMs = learn.similarityMs;
    sampleViewsRho = learn.sampleViewsRho;
    sampleViewsTheta = learn.sampleViewsTheta;
    sampleViewsPhi = learn.sampleViewsPhi;
    sampleRViewsTheta = learn.sampleRViewsTheta;
    sampleRViewsPhi = learn.sampleRViewsPhi;
    nOverlap = learn.nOverlap;
    dist = learn.dist;

  return *this;
}

apLearn::~apLearn() {
	// TODO Auto-generated destructor stub
}
