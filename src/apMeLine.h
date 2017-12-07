/*
 * apMeLine.h
 *
 *  Created on: Jul 7, 2012
 *      Author: agpetit
 */

#ifndef APMELINE_H_
#define APMELINE_H_

#include <visp/vpConfig.h>
#include <visp/vpMe.h>
#include <visp/vpPoint.h>
//#include <visp/vpMeLine.h>
//#include <visp/vpMbtMeLine.h>
#include <visp/vpMeTracker.h>
#include "apMHMeTracker.h"

class VISP_EXPORT apMeLine : public apMHMeTracker//: public vpMbtMeLine
{

private:
    vpMeSite PExt[2] ;
    double rho, theta, theta_1;
    double delta ,delta_1;
    int sign;

public:
    double a,b,c;
  int imin, imax;
  int jmin, jmax;
  double expecteddensity;
public:
	apMeLine();
	void InitLine(double a2, double b2, double c2);
	apMeLine(double a2, double b2, double c2);
	~apMeLine();
	apMeLine& operator =(apMeLine& f);
	void setRho(double _rho){rho = _rho;}
	void setTheta(double _theta){theta = _theta;}
	double getRho(){return rho;}
	double getTheta(){return theta;}

    void initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho, double theta);
    void track(const vpImage<unsigned char> &I);
    void updateParameters(const vpImage<unsigned char> &I, double rho, double theta);
    void updateParameters(const vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2, double rho, double theta);
    void display(const vpImage<unsigned char>& /*I*/, vpColor /*col*/){;}
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

   void set_a(double _a){a = _a;}
   void set_b(double _b){b = _b;}
   void set_c(double _c){c = _c;}

private:
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

};

#endif /* APMELINE_H_ */
