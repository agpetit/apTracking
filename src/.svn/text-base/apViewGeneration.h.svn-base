#ifndef apViewGeneration_h
#define apViewGeneration_h

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpImage.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpList.h>
#include "vpAROgre.h"
#include "apOgre.h"



class apViewGeneration
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
std::vector< vpImage<unsigned char>* > EdgeOrientMaps;
std::vector< vpImage<double>* > DTTemp;
std::vector< vpImage<vpRGBa>* > argDTTemp;
//std::vector< std::vector<vpImage<vpRGBa>*> > argDTTempNbhd;
std::vector< vpImage<vpRGBa>* > mergeTemp;
std::vector< vpImage<vpRGBa>* > Ipyramid;

public:
apViewGeneration();
virtual ~apViewGeneration();
void init();
void init(int s_Rx, int s_Ry, int s_Rz, double s_Z);
void initSph(int s_Rho, int s_Theta, int s_Phi);
void initSphNbhd(int s_Rho, int s_Theta, int s_Phi, int nb_Theta, int nb_Phi);
//void computeGradOrientations(vpImage<unsigned char> &I); 
void createViews(vpAROgre &ogre, vpPoseVector PosInit, std::string opath);
void createViewsDT(vpAROgre &ogre, vpPoseVector PoseInit, std::string opath);
void createViewsSph(vpAROgre &ogre, vpPoseVector PoseInit, std::string opath);
void createViewsSphNbhd(vpAROgre &ogre, vpPoseVector PoseInit, std::string opath);
void invert(vpImage<unsigned char> &IT, vpImage<unsigned char> &Irot);
void distanceTransform(vpImage<unsigned char> &I0,vpImage<unsigned char> &I1);
void computeSimilarity(vpImage<unsigned char> &I0);
void computeSimilarity0(vpImage<unsigned char> &I0, std::string opath);
void detect(vpImage<unsigned char> &I0, std::string opath);
void detect1(vpImage<unsigned char> &I0, std::string opath);
void rotateTemp( vpImage<unsigned char> &IT, double angle);
void detect2(vpImage<unsigned char> &I0, std::string opath);
void detectMS(vpImage<unsigned char> &I0, std::string opath);
void detectMS1(vpImage<unsigned char> &I0, std::string opath);
double computeSimilarity(vpImage<vpRGBa> *I0,vpImage<vpRGBa> *I1);
double computeSimilarityScale(vpImage<vpRGBa> *I0,vpImage<vpRGBa> *I1, int scale);
void computeSimilarityMatrix(const char *filename);
vpImage<double> *dt(vpImage<unsigned char> *im,  vpImage<vpRGBa> *imArg);
vpImage<vpRGBa> *dt0(vpImage<unsigned char> *im);
void dtf(vpImage<double> *im,  vpImage<vpRGBa> *imArg);
void dtf0(vpImage<double> *im,  vpImage<vpRGBa> *imArg);
float *dty(float *f, float *g, int n);
float *dtx(float *f,float *g, int n, int *argx, int *argy);
void dT(vpImage<unsigned char> &I0, vpImage<double> &I1, vpImage<vpRGBa> &I2);
//void dT0(vpImage<unsigned char> &I0, vpImage<vpRGBa> &I2);
void dTTemplates();
void dTTemplates0();
void buildHierarchy(double threshold, const char *filename);
void buildHierarchy1(double threshold, const char *filename);
void edgeOrientMap(vpImage<unsigned char> &I0);
void merge();
vpImage<vpRGBa> *merge(vpImage<vpRGBa> *I0,vpImage<vpRGBa> *I1);
vpImage<vpRGBa>* mergeScale(vpImage<vpRGBa> *I0, vpImage<vpRGBa> *I1, int scale);
double computeSimilarity(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT);
double computeSimilarityPos(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT,vpImagePoint pI);
void buildHierarchyMS(double threshold, const char *filename);
vpImage<vpRGBa> *downScale(int i, vpImage<vpRGBa> *_I);
void downScale(vpImage<unsigned char> &_I, int i);
void initPyramid(vpImage<vpRGBa>* _I, std::vector< vpImage<vpRGBa>* >& _pyramid);
};



#endif
