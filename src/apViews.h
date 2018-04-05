#ifndef apViews_h
#define apViews_h

#include "surrender/scenemanager.h"
#include "surrender/sceneviewer.h"

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpList.h>
//#include <visp3/gui/vpDisplayX.h>
#include <visp/vpCameraParameters.h>
//#include <GL/glew.h>
//#include <GL/glext.h>
//#include <GL/gl.h>
//#include "vpAROgre.h"
#include "apImageFilter.h"
#include "apContourPoint.h"
#include "apLearn.h"

#include <cstdlib>
#include "luaconfig.h"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
//using namespace cv;

using namespace luxifer;




class apViews
{

protected:
  
int sample_Rx;
int sample_Ry;
int sample_Rz;
int sample_Z;
int sample_Rho;
int sample_Theta;
int sample_Phi;
int nbhd_Theta;
int nbhd_Phi;
int snbhd_Theta;
int snbhd_Phi;
int overlap;
double dist;
std::vector< vpImage<unsigned char>* > EdgeOrientMaps;
std::vector< vpImage<double>* > DTTemp;
std::vector< vpImage<vpRGBa>* > argDTTemp;
std::vector< vpImage<vpRGBa>* > mergeTemp;
std::vector< vpImage<vpRGBa>* > Ipyramid;
std::vector< std::vector<vpImage<unsigned char>*>*> Hviews;
vpMatrix transProb;

double cannyTh1;
double cannyTh2;
double lambdaO;
double muD;


std::string object;

//vpAROgre Ogre;

public:
apViews();
virtual ~apViews();
void init();
void initViewSphere(apLearn &_learn, std::string _object);
void initDetection(apDetection &_detect);
void rotateView( vpImage<unsigned char> &IT,  vpImage<unsigned char> &Irot, double angle);
void invert(vpImage<unsigned char> &IT, vpImage<unsigned char> &Irot);
void invert90(vpImage<unsigned char> &IT, vpImage<unsigned char> &Irot);
void centerView(vpImage<unsigned char> &Itemp, vpImagePoint &cog);
void edgeOrientMap(vpImage<unsigned char> &I0);
int edgeOrientMapN(vpImage<unsigned char> &I0);
vpImage<vpRGBa>* dTO(vpImage<unsigned char> &I0);
vpImage<vpRGBa>* dtOSeg(vpImage<unsigned char> &Iseg);
vpImage<vpRGBa>* dtOSeg2(vpImage<unsigned char> &Iseg);
vpImage<vpRGBa>* dtOSeg3(vpImage<unsigned char> &Iseg, int &nedge);
vpImage<double> *dt(vpImage<unsigned char> *im,  vpImage<vpRGBa> *imArg);
vpImage<vpRGBa> *dt0(vpImage<unsigned char> *im);
void dtf(vpImage<double> *im,  vpImage<vpRGBa> *imArg);
void dtf0(vpImage<double> *im,  vpImage<vpRGBa> *imArg);
float *dty(float *f, float *g, int n);
float *dtx(float *f,float *g, int n, int *argx, int *argy);
void dT(vpImage<unsigned char> &I0, vpImage<double> &I1, vpImage<vpRGBa> &I2);
void bin(vpImage<unsigned char> &I1);
void computePosOri(vpImage<unsigned char> &I1, vpImagePoint &cog,double &angle, int &surface);
void computePosOriMean(vpImage<unsigned char> &I1, vpImagePoint &cog,double &angle, int &surface);
void computePosOriMean2(vpImage<unsigned char> &I1, vpImagePoint &cog,double &angle, int &surface);
void computeDotView(vpImage<unsigned char> &Itemp, vpImagePoint &cog,double &angle, int &surface);
void computeDotView2(vpImage<unsigned char> &Itemp, vpImagePoint &cog,double &angle, int &surface);
void computeSimilarityMatrix(vpMatrix &simmatrix, vpMatrix &preferences);
std::vector<double> computeSC(vpImage<unsigned char> &Iseg, vpImage<unsigned char> &IsegEdge, vpImagePoint &cog, double &angle, int &surface, int nr, int nw);
void buildClusters(unsigned long *idx, const int maxits, const int convits, const double lam);
void buildViewGraph(vpCameraParameters &mcam, SceneManager *mgr, std::string opath, int height, int width);
void loadViews(std::string opath);
void MSERs(vpImage<unsigned char> &I0);
double computeSimilarity(vpImage<vpRGBa> *I0,vpImage<vpRGBa> *I1);
double computeSimilarityScale(vpImage<vpRGBa> *I0,vpImage<vpRGBa> *I1, int scale);
double computeSimilarity(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT);
double computeSimilarityPos(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT,vpImagePoint pI);
double computeSimilarityPosRot(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT, vpImagePoint &pI, const double Rot);
double computeSimilarityPosRotScale(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT, vpImagePoint &pI, const double Rot);
double computeSimilarityCP(vpImage<vpRGBa> *Im, std::vector<apContourPoint*> &ContourPoints, vpImagePoint &pI, const double Rot);
double computeSimilarityCPOpt(vpImage<vpRGBa> *Im, std::vector<apContourPoint*> &ContourPoints, vpImagePoint &pI, const double Rot);
double computeSimilarityCPSOpt(vpImage<vpRGBa> *Im, std::vector<apContourPoint*> &ContourPoints, const int xpI, const int ypI, const double Rot, const double scale);
double computeSimilarityCPSOptPartial(vpImage<vpRGBa> *Im, std::vector<apContourPoint*> &ContourPoints, const int xpI, const int ypI, const double Rot, const double scale);
double computeSimilaritySteger(vpImage<unsigned char> &Igrad, std::vector<apContourPoint*> &ContourPoints, const int xpI, const int ypI, const double Rot, const double scale);
double computeSimilarityPosRotScaleSeg(vpImage<vpRGBa> *Im, vpImage<vpRGBa> *Iseg, vpImage<unsigned char> *IT, vpImagePoint &pI, const double Rot);
double computeSimilaritySC(std::vector<double> &CP0, std::vector<double> &CP1);
double computeSimilaritySCPerm(std::vector<double> &CP0, std::vector<double> &CP1, int perm, int nr, int nw);
vpImage<vpRGBa> *downScale(int i, vpImage<vpRGBa> *_I);
void downScale(vpImage<unsigned char> &_I, int i);
void initPyramid(vpImage<vpRGBa>* _I, std::vector< vpImage<vpRGBa>* >& _pyramid);
};



#endif
