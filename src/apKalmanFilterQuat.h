/*
 * apKalmanFilterQuat.h
 *
 *  Created on: May 3, 2013
 *      Author: agpetit
 */

#ifndef APKALMANFILTERQUAT_H_
#define APKALMANFILTERQUAT_H_

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpPose.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpMatrixException.h>
#include <visp/vpMatrix.h>
#include <visp/vpImagePoint.h>

#include <visp/vpException.h>
#include <visp/vpTrackingException.h>
#include "apKalmanFilter.h"


class VISP_EXPORT apKalmanFilterQuat {
public:
	vpHomogeneousMatrix cMoPred;
	vpHomogeneousMatrix cMoPred_0;
	vpHomogeneousMatrix cMoEst;
	vpHomogeneousMatrix cMoEst_0;
	vpHomogeneousMatrix cMoInnov;
	vpColVector vPred;
	vpColVector vEst;
	vpColVector vMes;
	vpColVector XEst;
	vpColVector XPred;
	vpColVector XMes;
	vpMatrix PEst;
	vpMatrix PPred;
	vpMatrix PvEst;
	vpMatrix PvPred;
	vpMatrix Q;
	vpMatrix Qv;
	vpMatrix R;
	vpMatrix Rv;
	vpMatrix J;
	vpMatrix K;
	vpMatrix Kv;
	vpMatrix H;
	vpMatrix H_0;
	apKalmanFilterQuat();
	void initFilter(vpHomogeneousMatrix &cMo, apKalmanParameters &kalmanParam);
	virtual ~apKalmanFilterQuat();
	void predictPose();
	inline void getPredPose(vpHomogeneousMatrix &cMoPred_){cMoPred_ = cMoPred_0;}
	void estimatePose(vpHomogeneousMatrix &cMoMes, vpMatrix &covMes);
	void convert(vpHomogeneousMatrix &_cMo, vpColVector &poseQuat_);
	void convert(vpColVector &poseQuat_, vpHomogeneousMatrix &_cMo);
	void computeJacobian(double dt);
	vpMatrix getTheta(vpColVector &w);
	vpMatrix computeExp(vpMatrix &Theta, vpColVector &w);
	vpMatrix computePhi(vpColVector &w);

};

#endif /* APKALMANFILTERQUAT_H_ */
