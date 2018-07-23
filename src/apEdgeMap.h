#ifndef apEdgeMap_H
#define apEdgeMap_H

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <fstream>
#include <math.h>
#include <string.h>

#include <visp/vpException.h>
#include <visp/vpImageException.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <visp/vpParseArgv.h>
#include <visp/vpImage.h>
#include <visp/vpImageFilter.h>
#include <visp/vpMatrix.h>
#include <cv.h>
#include "apImageFilter.h"


#include <iostream>
using namespace std;

class apEdgeMap
{
protected :
	vpImage<double> Iuf;

public:
static void edgeMapLap(const vpImage<double> &I, vpImage<unsigned char> &If, double th);
static void edgeMapLap(const vpImage<unsigned char> &I, vpImage<unsigned char> &If, double th);
static void edgeMapLap(const vpImage<double> &I, const vpImage<unsigned char> &I1, vpMatrix &Ori, vpImage<unsigned char> &If, double th);
static void edgeMapLap2(const vpImage<unsigned char> &I1, vpMatrix &Ori, vpImage<unsigned char> &If, const double th);
static void edgeOrientMap(vpImage<unsigned char> &I0);
}
;
#endif
