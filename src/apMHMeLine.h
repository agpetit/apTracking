/*
 * apMHMeLine.h
 *
 *  Created on: Jul 6, 2012
 *      Author: agpetit
 */

#ifndef APMHMELINE_H_
#define APMHMELINE_H_
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>
#include <vector>

#include <visp/vpConfig.h>
#include <visp/vpColVector.h>
#include <visp/vpPoint.h>
#include <visp/vpMe.h>
#include <visp/vpMeTracker.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpImagePoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include "apMHMeTracker.h"
#include "apMHMeSite.h"
#include "apMeLine.h"
#include "apHoughVote.h"
#include "apControlPoint.h"

using namespace std;

class VISP_EXPORT apMHMeLine : public apMHMeTracker
{

 private:
    apMHMeSite PExt[2];
    double rho, theta, theta_1;
    double delta ,delta_1;
    int sign;
    double a,b,c;

  public:
    int imin, imax;
    int jmin, jmax;
    double expecteddensity;
    vpCameraParameters *cam;

    //! List of tracked points
    //std::vector<apMHMeSite> list ;
    std::vector< std::vector<apMHMeSite> > points_vect;//! Unused ?! Vector of the different vectors of sites (corresponding to possible lines)
  	//std::vector < vpMeLine >  lines_vect;
  	//std::vector< apMeLine >  lines_vect;
  	//vpList< vpImageLine >  lines_vect;
  	std::vector<float> weights;
  	std::vector<float> weight_cumul;

	apHoughVote LineVote;
	std::vector<double> Rho;
	std::vector<double> Theta;



public:
	apMHMeLine();
	~apMHMeLine();
	//int getNbOfCandidateLines() {return lines_vect.size();}
	apMHMeLine& operator =(apMHMeLine& f);
	void setCameraParameters(vpCameraParameters *_cam){ cam = _cam;}
    void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho, double theta);
    void initTrackingG(const vpImage<unsigned char> &I, const vpImage<unsigned char> &gradMap, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho, double theta);
    void initTrackingP(const vpImage<unsigned char> &I, std::vector<apControlPoint*> &points);
    void track(const vpImage<unsigned char> &I);
    void trackG(const vpImage<unsigned char> &I, const vpImage<unsigned char> &gradMap);
    void updateParameters(const vpImage<unsigned char> &I, double rho, double theta);
    void updateParameters(const vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2, double rho, double theta);
    void display(const vpImage<unsigned char>& /*I*/, vpColor /*col*/) {;}

     /*!
     Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$

     \return : The a coefficient of the moving edge
    */
    inline double get_a() const { return this->a;}

     /*!
     Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$

     \return : The b coefficient of the moving edge
    */
    inline double get_b() const { return this->b;}

     /*!
     Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$

     \return : The c coefficient of the moving edge
    */
    inline double get_c() const { return this->c;}

  private:
    int numberOfSignal_MH();
    double leastSquareLine(const vpImage<unsigned char> &I, std::vector<apMHMeSite> &tab,const vpCameraParameters &Cam,apMeLine &line,double &rho_m, double &theta_m);
    int findLinesByKmean(const vpImage< unsigned char> &I,int display_mode);
    void setParameterSpace(int &n, int &m, const vpImage<unsigned char> &I, vpCameraParameters &cam);
    int findLinesByKmean_2(const vpImage< unsigned char> &I,int display_mode);
    int findLinesByKmean_3(const vpImage< unsigned char> &I,int display_mode);
    void sample(const vpImage<unsigned char>&image);
    void reSample(const vpImage<unsigned char>&image);
    void reSample(const vpImage<unsigned char>&image, vpImagePoint ip1, vpImagePoint ip2);
    void updateDelta();
    void bubbleSortI();
    void bubbleSortJ();
    void suppressPoints(const vpImage<unsigned char> &I);
    void setExtremities();
    void seekExtremities(const vpImage<unsigned char> &I);
    void findSignal(const vpImage<unsigned char>& I, const vpMe *me, double *conv);
} ;

#endif /* APMHMELINE_H_ */
