/*
 * apKalmanFilterSE3.h
 *
 *  Created on: May 3, 2013
 *      Author: agpetit
 */

#ifndef APKALMANFILTERSE3_H_
#define APKALMANFILTERSE3_H_

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



class VISP_EXPORT apKalmanFilterSE3 {
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

	vpSE3Kalman filter;
	vpColVector Xv;
	vpColVector X;
	apKalmanFilterSE3();
	void initFilter(vpHomogeneousMatrix &cMo, apKalmanParameters &kalmanParam);
	virtual ~apKalmanFilterSE3();
	void predictPose();
	inline void getPredPose(vpHomogeneousMatrix &cMoPred_){cMoPred_ = cMoPred_0;}
	void estimatePose(vpHomogeneousMatrix &cMoMes, vpMatrix &covMes);

};


#endif /* APKALMANFILTERSE3_H_ */
