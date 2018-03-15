/*
 * apCCDTracker.h
 *
 *  Created on: Nov 23, 2012
 *      Author: agpetit
 */

#ifndef APCCDTRACKER_H_
#define APCCDTRACKER_H_


#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpPose.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageIo.h>
#include <visp/vpRobust.h>
#include <visp/vpMatrixException.h>
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRobust.h>

#ifdef True
#undef True
#undef False
#endif
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include "apControlPoint.h"

using namespace cv;
using namespace std;

struct apCCDParameters
{
  apCCDParameters(): gamma_1(0.5), gamma_2(4), gamma_3(4), gamma_4(3),alpha(1.3), beta(0.06), kappa(0.5),c(0.25), h(40), delta_h(1),resolution(100), degree(4), phi_dim(8)
  {
  }
  apCCDParameters(double p1,
            double p2,
            double p3,
            double p4,
            double p5,
            double p6,
            double p7,
            double p8,
            int p9,
            int p10,
            int p11,
            int p12,
            int p13
            )
  {
    gamma_1 = p1;
    gamma_2 = p2;
    gamma_3 = p3;
    gamma_4 = p4;
    alpha = p5;
    beta = p6;
    kappa = p7;
    c = p8;
    h = p9;
    delta_h = p10;
    resolution = p11;
    degree = p12;
    phi_dim = p13;
  }

  ~apCCDParameters()
  {
  }
  double gamma_1;
  double gamma_2;
  double gamma_3;
  double gamma_4;
  double alpha;
  double beta;
  double kappa;
  double c;
  int h;
  int delta_h;
  int resolution;
  int degree;
  int phi_dim;
  int fixedrotationx;
  int fixedrotationy;
  int fixedrotationz;

};

class VISP_EXPORT apCCDTracker
{
public:
	std::vector< std::vector<apControlPoint*> > pointsCCD;
	apCCDParameters ccdParameters;
	vpCameraParameters cam;
	cv::Mat vic;
	cv::Mat vic_old;
	cv::Mat mean_vic;
	cv::Mat cov_vic;
	cv::Mat nv;

	cv::Mat vic_prev;
	cv::Mat mean_vic_prev;
	cv::Mat cov_vic_prev;
	cv::Mat nv_prev;

	cv::Mat Phi;
	cv::Mat Sigma_Phi;
	cv::Mat delta_Phi;
	cv::Mat p_old;
	cv::Mat nabla_E;
	cv::Mat hessian_E;
	cv::Mat hessian_E_partial;
	vpColVector w_ccd;
	cv::Mat image;
	cv::Mat image_prev;
	int npointsCCD;
	int scaleLevel;
	int resolution;
	int nerror_ccd;

    double tol;
    double tol_old;
    bool convergence;
    double norm;
    vpMatrix sigmaF;
    vpMatrix sigmaP;

    vpColVector error_ccd;
    vpColVector error0_ccd;
    vpColVector weighted_error_ccd;

    vpMatrix L_ccd;
    vpMatrix L_ccd_partial;


	apCCDTracker();
	virtual ~apCCDTracker();
	void init(apCCDParameters &_ccdParameters, vpCameraParameters &_cam);
	void setImage(const vpImage<vpRGBa> &_I);
	void setPrevImage(const vpImage<vpRGBa> &_I);
	void setOld();
	//void initRobust();
	void setCCDPoints(std::vector< std::vector<apControlPoint*> > &_pointsCCD){pointsCCD = _pointsCCD;}
	void setScaleLevel(int _scaleLevel){scaleLevel = _scaleLevel;}
	double getResolution(){return ccdParameters.resolution;}
	void clearCCDTracker();
	void updateCCDPoints(vpHomogeneousMatrix &cMo);
	void computeLocalStatistics();
	void computeLocalStatisticsPrev(const vpImage<unsigned char> &I);
	void computeLocalStatisticsPrev0();
	void computeLocalStatisticsPrev1();

	void computeLocalStatisticsSpace();
	void computeLocalStatisticsPrevSpace();
	void updateParameters(vpMatrix &LTCIL, vpColVector &LTCIR);
	void updateParametersPrev(vpMatrix &LTCIL, vpColVector &LTCIR);
	void updateParametersRobust(vpMatrix &LTCIL, vpColVector &LTCIR, vpRobust &robust);
	void updateParametersRobustPrev(vpMatrix &LTCIL, vpColVector &LTCIR, vpRobust &robust);
	void checkCCDConvergence();
	void choleskyDecomposition(cv::Mat &A, cv::Mat &L,int n);
	vpMatrix computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b);
};

#endif /* APCCDTRACKER_H_ */
