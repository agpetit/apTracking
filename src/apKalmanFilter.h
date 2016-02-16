/*
 * apKalmanFilter.h
 *
 *  Created on: Nov 7, 2012
 *      Author: agpetit
 */

#ifndef APKALMANFILTER_H_
#define APKALMANFILTER_H_

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpPose.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpMatrixException.h>
#include <visp/vpMatrix.h>
#include <visp/vpImagePoint.h>

#include <visp/vpException.h>
#include <visp/vpTrackingException.h>

struct apKalmanParameters
{
 apKalmanParameters(): sigmaPT(0.0001), sigmaPR(0.0001), sigmaQT(0.3), sigmaQR(0.6)
  {
  }
 apKalmanParameters(double p1,
            double p2,
            double p3,
            double p4
            )
  {
	sigmaPT = p1;
	sigmaPR = p2;
	sigmaQT = p3;
	sigmaQR = p4;
  }

  ~apKalmanParameters()
  {
  }
  double sigmaPT;
  double sigmaPR;
  double sigmaQT;
  double sigmaQR;
};

class VISP_EXPORT apKalmanFilter {
public:
	vpHomogeneousMatrix cMoPred;
	vpHomogeneousMatrix cMoPred_0;
	vpHomogeneousMatrix cMoEst;
	vpHomogeneousMatrix cMoEst_0;
	vpHomogeneousMatrix cMoInnov;
	vpColVector vPred;
	vpColVector vEst;
	vpColVector vMes;
	vpMatrix PEst;
	vpMatrix PPred;
	vpMatrix PvEst;
	vpMatrix PvPred;
	vpMatrix Q;
	vpMatrix Qv;
	vpMatrix R;
	vpMatrix Rv;
	vpMatrix J;
	vpMatrix Jv;
	vpMatrix K;
	vpMatrix Kv;
	vpMatrix H;
	vpMatrix H_0;
	vpMatrix L;
	apKalmanFilter();
	void initFilter(vpHomogeneousMatrix &cMo, apKalmanParameters &kalmanParam);
	virtual ~apKalmanFilter();
	void predictPose();
	inline void getPredPose(vpHomogeneousMatrix &cMoPred_){cMoPred_ = cMoPred_0;}
	void estimatePose(vpHomogeneousMatrix &cMoMes, vpMatrix &covMes);

};

#endif /* APKALMANFILTER_H_ */
