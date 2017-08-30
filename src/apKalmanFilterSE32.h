/*
 * apKalmanFilterSE32.h
 *
 *  Created on: May 28, 2013
 *      Author: agpetit
 */

#ifndef APKALMANFILTERSE32_H_
#define APKALMANFILTERSE32_H_

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpPose.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpMatrixException.h>
#include <visp/vpMatrix.h>
#include <visp/vpImagePoint.h>

#include <visp/vpException.h>
#include <visp/vpTrackingException.h>
#include "vpSE3Kalman.h"
#include "apKalmanFilter.h"

class VISP_EXPORT apKalmanFilterSE32 {
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
	vpMatrix F;
	vpMatrix Jn;
	vpMatrix Jv;
	vpMatrix K;
	vpMatrix Kv;
	vpMatrix H;
	vpMatrix H_0;
	vpMatrix L;
	apKalmanFilterSE32();
	void initFilter(vpHomogeneousMatrix &cMo, apKalmanParameters &kalmanParam);
	virtual ~apKalmanFilterSE32();
	void predictPose();
	inline void getPredPose(vpHomogeneousMatrix &cMoPred_){cMoPred_ = cMoPred_0;}
	void estimatePose(vpHomogeneousMatrix &cMoMes, vpMatrix &covMes);
	void computeJacobian(double dt);

};

#endif /* APKALMANFILTERSE32_H_ */
