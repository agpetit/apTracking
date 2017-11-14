/*
 * apKalmanFilterSE3.cpp
 *
 *  Created on: May 3, 2013
 *      Author: agpetit
 */

#include "apKalmanFilterSE3.h"

apKalmanFilterSE3::apKalmanFilterSE3() {
	// TODO Auto-generated constructor stub

}

apKalmanFilterSE3::~apKalmanFilterSE3() {
	// TODO Auto-generated destructor stub
}

void apKalmanFilterSE3::initFilter(vpHomogeneousMatrix &cMo, apKalmanParameters &kalmanParam)
{

	double sQT = kalmanParam.sigmaQT;
	double sQR = kalmanParam.sigmaQR;
	double sPT = kalmanParam.sigmaPT;
	double sPR = kalmanParam.sigmaPR;
	vEst.resize(6);
	Q.resize(12,12);
	R.resize(6,6);
	J.resize(12,12);
	H.resize(6,12);
	H_0.resize(6,12);
	PEst.resize(12,12);
	J.setIdentity();
	Q.setIdentity();
	L.resize(6,12);
	Q =sQT*sQT*Q;
	PEst.setIdentity();
	PEst = sPT*sPT*PEst;

	K.resize(12,6);

	Qv.resize(6,6);
	Rv.resize(6,6);
	Jv.resize(6,6);
	Kv.resize(6,6);
	Kv.setIdentity();
	PvEst.resize(6,6);
	Jv.setIdentity();
	Qv.setIdentity();
	Rv.setIdentity();
	Qv =sQR*sQR*Qv;
	PvEst.setIdentity();
	PvEst = sPR*sPR*PvEst;


	for(int j = 0; j<12; j++)
		for(int i = 0; i<12; i++)
		{
			//R[i][j] = 0.0001;
			if(i>=6 && j<6)
				{
				if(j==i-6)
					{
					K[i][j] = 1;
					}
				}
			if(i<6 && j>=6)
				{
				if(i==j-6)
					{J[i][j] = 1;
					H_0[i][j] = 1;
					}

				L[i][j] = 1;
				}
			else if(j<6 && i<6)
				{
				Q[i][j] = 0;
				if(i == j)
				{H[i][j] = 1;
				R[i][j] = 0.02;
				Rv[i][j] = 0.00000001;}
				if (j<3 && i<3){
					if(i == j)
						{Qv[i][j] = sQT*sQT;
						PvEst[i][j] = sPT*sPT;
						//R[i][j] = 3e-09;
						R[i][j] = 0.001;
						Rv[i][j] = 0.0000001;
						}
				}
				}
		}
	Q[9][9] = sQR*sQR;
	Q[10][10] = sQR*sQR;
	Q[11][11] = sQR*sQR;
	cMoEst = cMo;
	cMoEst_0 = cMo;
	vEst[0] = 0;
	vEst[1] = 0;
	vEst[2] = 0;
	vEst[3] = 0;
	vEst[4] = 0;
	vEst[5] = 0;
	vMes = vEst;
	vPred=vMes;

	vpTranslationVector tr;
	vpRotationMatrix R0;
	vpThetaUVector r;
	cMo.extract(tr);
	cMo.extract(R0);
	r.buildFrom(R0);
	Xv.resize(6);
	Xv[0] = tr[0];
	Xv[1] = tr[1];
	Xv[2] = tr[2];
	Xv[3] = r[0];
	Xv[4] = r[1];
	Xv[5] = r[2];
	X.resize(12);
	//f >> iter >> x >> y>> z>>beta>>gamma>>theta;
	/*Qv.setIdentity();
	Qv *= 0.1;*/
	filter.setXv(Xv);
	filter.setQv(R);
	filter.setQu(Qv);
	filter.init();


}

void apKalmanFilterSE3::predictPose()
{
	filter.predict(1);
	X=filter.getState();
	std::cout << " Xpred " << X << std::endl;

	vPred[0] = X[6];
	vPred[1] = X[7];
	vPred[2] = X[8];
	vPred[3] = X[9];
	vPred[4] = X[10];
	vPred[5] = X[11];
	vpMatrix Iv(6,6);
	Iv.setIdentity();
	vpMatrix I(12,12);
	I.setIdentity();
	//cMoPred = vpExponentialMap::direct(vPred).inverse() * cMoEst;
	cMoPred_0 =  cMoEst;
	//std::cout << " Q " << H_0*K << std::endl;

	//cMoPred_0 = vpExponentialMap::direct((Iv-H_0*K)*vPred).inverse() * cMoEst;
}

void apKalmanFilterSE3::estimatePose(vpHomogeneousMatrix &cMoMes, vpMatrix &covMes)
{
	std::cout << " covMes " << covMes << std::endl;
	vpTranslationVector tr;
	vpRotationMatrix R;
	vpThetaUVector r;
	cMoMes.extract(tr);
	cMoMes.extract(R);
	r.buildFrom(R);
	Xv[0] = tr[0];
	Xv[1] = tr[1];
	Xv[2] = tr[2];
	Xv[3] = r[0];
	Xv[4] = r[1];
	Xv[5] = r[2];
	filter.setXv(Xv);
	/*covMes.setIdentity();
	covMes*=0.05;*/
	filter.setQv(covMes);
	//filter.setQv(Qv);
	filter.visionUpdate();
	K = filter.getGain();
	std::cout << " covMes " << K << std::endl;

	X=filter.getState();
	/*R[0][0] = X[3];
	R[0][1] = X[4];
	R[0][2] = X[5];
	R[1][0] = X[6];
	R[1][1] = X[7];
	R[1][2] = X[8];
	R[2][0] = X[9];
	R[2][1] = X[10];
	R[2][2] = X[11];*/
	tr[0] = X[0];
	tr[1] = X[1];
	tr[2] = X[2];
	r[0] = X[3];
	r[1] = X[4];
	r[2] = X[5];
	R.buildFrom(r);
	cMoEst.buildFrom(tr,R);
}
