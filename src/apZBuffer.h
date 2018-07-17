
#ifndef apZBuffer_H
#define apZBuffer_H

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
#include <visp/vpDisplayX.h>
#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif 
//#include <GL/glu.h>
#include <visp/vpMatrix.h>
#include <visp/vpDisplay.h>

#include <iostream>
using namespace std;

class vpDisplay;
class vpMatrix;

class VISP_EXPORT apZBuffer : public vpMatrix
{
protected :
	vpMatrix ZCoord;
	double znear;
	double zfar;

public:

	int borni1;
	int borni2;
	int bornj1;
	int bornj2;

void init();
//void init(const vpImage<unsigned char> &I);

apZBuffer();
//apZBuffer(const vpImage<unsigned char> &I);
apZBuffer(const apZBuffer &B);
void set(const int r, const int c, const double znear, const double zfar);
void getDepth(vpImage<double> &I,vpImage<unsigned char> &Ic);
void getLuminance(vpImage<double> &I,vpImage<unsigned char> &Ic);
void getDepth(vpImage<unsigned char> &Ic);
void cast(vpImage<unsigned char> &Id0, vpImage<unsigned char> &Id,vpImage<unsigned char> &Ic);
void getCastLuminance(vpImage<unsigned char> &Ic);
void getDepth();
void dImage(vpImage<double> &I);
void displayZB(vpImage<unsigned char> &I);
void getCastRGBLuminance(vpImage<unsigned char> &Ic,vpImage<vpRGBa> &Ip,vpMatrix &Zcoord,const double dist);
void getRGBDepth(vpImage<double> &Ic, vpImage<unsigned char> &Ic1, vpImage<unsigned char> &Ic2, vpImage<vpRGBa> &Ip,vpMatrix &Zcoord,const double dist, const double &m);
vpMatrix getZ(const double dist);

};

#endif
