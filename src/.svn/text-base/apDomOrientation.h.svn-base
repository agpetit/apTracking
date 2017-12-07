#ifndef apDomOrientation_h
#define apDomOrientation_h

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpImage.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpList.h>



class apDomOrientation
{
 protected:
  
int region_size;
int nbor;
vpMatrix Ori;
std::vector< vpColVector > Orientations;
std::vector< vpColVector > OrientationsTemp;
double threshold;
vpImage<double> imIx, imIy;
int nbar;
int nb_regioni;
int nb_regionj;
int nb_region;
int nb_regionTi;
int nb_regionTj;
int nb_regionT;
int nb_regionIi;
int nb_regionIj;
int nb_regionI;
//apOriMat OriMat;
//apOriMat OriMatTemp;




public:
apDomOrientation();
virtual ~apDomOrientation();
void init();
void init(int r_size, int k, int nb, double th);
//void computeGradOrientations(vpImage<unsigned char> &I); 
void buildFrom(vpImage<unsigned char> &I,vpImage<unsigned char> &Iout, unsigned int u);
vpColVector computeCostFunction();
std::vector< vpColVector > getDominantOrientations(){return Orientations;}



} ;



#endif
