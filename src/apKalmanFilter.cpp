/*
 * apKalmanFilter.cpp
 *
 *  Created on: Nov 7, 2012
 *      Author: agpetit
 */

#include "apKalmanFilter.h"

apKalmanFilter::apKalmanFilter() {
	// TODO Auto-generated constructor stub
}

apKalmanFilter::~apKalmanFilter() {
	// TODO Auto-generated destructor stub
}

void apKalmanFilter::initFilter(vpHomogeneousMatrix &cMo, apKalmanParameters &kalmanParam)
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
                                R[i][j] = 0.0002;
                                Rv[i][j] = 0.00001;}
				if (j<3 && i<3){
					if(i == j)
						{Qv[i][j] = sQT*sQT;
						PvEst[i][j] = sPT*sPT;
						//R[i][j] = 3e-09;
                                                R[i][j] = 0.001;
                                                Rv[i][j] = 0.0001;
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


}

void apKalmanFilter::predictPose()
{
	vPred = vEst;
	vpMatrix Iv(6,6);
	Iv.setIdentity();
        vpMatrix I(12,12);
        I.setIdentity();
        cMoPred = vpExponentialMap::direct(vPred).inverse() * cMoEst;
	//cMoPred_0 = vpExponentialMap::direct((Iv)*vPred).inverse() * cMoEst;
        //cMoPred_0 = vpExponentialMap::direct(vMes-vPred).inverse() * cMoEst;
        cMoPred = cMoEst;
	cMoPred_0 = cMoEst;
	PPred = J*PEst*J.transpose() + Q;
    PvPred = PvEst + Qv;
    cMoEst_0 = cMoEst;
}

void apKalmanFilter::estimatePose(vpHomogeneousMatrix &cMoMes, vpMatrix &covMes, bool printLog)
{
    cMoInnov = cMoMes*(cMoPred.inverse());
    K = PPred*H.transpose()*((H*PPred*H.transpose() + covMes).pseudoInverse());
    //Kv = PvPred*((PvPred + R).pseudoInverse());
    Kv = PvPred*((PvPred + covMes).pseudoInverse());
    vpMatrix Iv(6,6);
    Iv.setIdentity();
    vMes = vpExponentialMap::inverse((cMoInnov));

    if(printLog)
        std::cout << " Kv " << Kv << std::endl;
    vEst = vPred + (Kv)*(vMes-vPred);
    //vEst = vPred + (H_0)*K*(vMes-vPred);
    //cMoEst = vpExponentialMap::direct(Kv*(vMes-vPred)).inverse()*cMoPred;
    //cMoEst = vpExponentialMap::direct(K*vpExponentialMap::inverse(cMoInnov.inverse())).inverse()*cMoPred;
    cMoEst = vpExponentialMap::direct(vEst)*cMoEst;
    //cMoEst = cMoMes;

    //PEst = (I-K*H)*PPred;
    PvEst = (Iv-Kv)*PvPred;
}
