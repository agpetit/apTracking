/*
 * apRend.cpp
 *
 *  Created on: May 10, 2011
 *      Author: Antoine Petit
 */

#include <stdlib.h>


#include "apRend.h"
#include <visp/vpColVector.h>
#include <visp/vpMath.h>


apRend::apRend()
{
  edgeR_th = 0.1 ;
  clipDist = 1;
  sampleR = 5;
  scaleModel = 1;
  Normx = 1;
  Normy = 1;
  Normz = 1;
  nPoints = 0;
  useNPoints = 0;

}

apRend::apRend(const apRend &rend)
{
  *this = rend;
}

const
apRend& apRend::operator=(const apRend &rend)
{
	edgeR_th = rend.edgeR_th ;
	clipDist = rend.clipDist;
	sampleR = rend.sampleR;
	scaleModel = rend.scaleModel;
	Normx = rend.Normx;
	Normy = rend.Normy;
    Normz = rend.Normz;
    nPoints = rend.nPoints;
    useNPoints = rend.useNPoints;
  return *this;
}

apRend::~apRend()
{

}






