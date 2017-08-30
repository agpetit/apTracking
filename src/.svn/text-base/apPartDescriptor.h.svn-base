/*
 * apPartDescriptor.h
 *
 *  Created on: Dec 19, 2011
 *      Author: agpetit
 */

#ifndef APPARTDESCRIPTOR_H_
#define APPARTDESCRIPTOR_H_

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpImage.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpList.h>
#include "BOMatrix.h"
#include "BOLogPolarHist.h"



class apPartDescriptor
{
 protected:

int width;
int height;
vector<double> xcoord;
vector<double> ycoord;
vector<double> Descriptor;
vpMatrix Historgram;
int nR;
int nW;


public:
apPartDescriptor();
virtual ~apPartDescriptor();
void init(int w, int h);
void buildFrom(vpImage<unsigned char> EOMap, vpImagePoint PointO);

};



#endif


#endif /* APPARTDESCRIPTOR_H_ */
