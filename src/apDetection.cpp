/*
 * apDetection.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: agpetit
 */
#include <stdlib.h>
#include "apDetection.h"

apDetection::apDetection() {
	// TODO Auto-generated constructor stub
    similarityMs = ORIENTED_CHAMFER;
    nbParticles = 100;
    nr = 20;
    nw = 40;
    nscales = 10;
    sx = 20;
    sy = 20;
    stheta = 1;
    nbimax = 10;
    startingLevel = 1;
    cannyTh1 = 150;
    cannyTh2 = 100;
    lambda = 300;
    mud = 400;
    lambdao = 0.1;
    sigmaf = 0.1;
    sample = 1;
}
apDetection::apDetection(const apDetection &detect)
{
  *this = detect;
}

const
apDetection& apDetection::operator=(const apDetection &detect)
{
    similarityMs = detect.similarityMs;
    nbParticles = detect.nbParticles;
    nr = detect.nr;
    nw = detect.nw;
    nscales = detect.nscales;
    sx = detect.sx;
    sy = detect.sy;
    stheta = detect.stheta;
    nbimax = detect.nbimax;
    startingLevel = detect.startingLevel;
    cannyTh1 = detect.cannyTh1;
    cannyTh2 = detect.cannyTh2;
    lambda = detect.lambda;
    mud = detect.mud;
    lambdao = detect.lambdao;
    sigmaf = detect.sigmaf;
    sample = detect.sample;

  return *this;
}

apDetection::~apDetection() {
	// TODO Auto-generated destructor stub
}
