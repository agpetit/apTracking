#ifndef apLineExtractor_H
#define apLineExtractor_H

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <fstream>
#include <math.h>
#include <string.h>

#include <visp/vpException.h>
#include <visp/vpImageException.h>
#include <visp/vpDisplay.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <visp/vpParseArgv.h>
#include <visp/vpImage.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpImagePoint.h>
#include <visp/vpMe.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMbtMeLine.h>
#include <visp/vpMbtDistanceLine.h>
#include <visp/vpColor.h>
#include <visp/vpCameraParameters.h>
#include <vector>


#include "apHoughVote.h"
#include "apZBuffer.h"

#include <iostream>

class VISP_EXPORT apLineExtractor
{

protected :
vpColVector Theta;
vpColVector Rho;

std::vector < vpImagePoint> points;
std::vector< vpMbtDistanceLine* > lines;
int nline;
apHoughVote V;
vpMe me;
vpMatrix Zc;


public:

void setParameterSpace(const int n, const int m, vpImage<unsigned char> &I);
void setLinePoints(const vpImage<unsigned char> &I,const int th, const apZBuffer zbuf, vpCameraParameters &cam);
void buildHoughVote(const vpImage<unsigned char> &I);
void setFeaturePoints(const vpImage<unsigned char> &I);
//void setLines(vector<Vec4i> &Lines, const apZBuffer &zbuf, vpCameraParameters &cam);
vpMatrix getVMatrix();
//std::vector< vpMbtDistanceLine* > getLines(){return lines};

}
;
#endif
