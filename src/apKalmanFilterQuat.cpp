/*
 * apKalmanFilterQuat.cpp
 *
 *  Created on: May 3, 2013
 *      Author: agpetit
 */

#include "apKalmanFilterQuat.h"

apKalmanFilterQuat::apKalmanFilterQuat() {
	// TODO Auto-generated constructor stub

}

apKalmanFilterQuat::~apKalmanFilterQuat() {
	// TODO Auto-generated destructor stub
}

void apKalmanFilterQuat::initFilter(vpHomogeneousMatrix &cMo, apKalmanParameters &kalmanParam)
{

	double sQT = kalmanParam.sigmaQT;
	double sQR = kalmanParam.sigmaQR;
	double sPT = kalmanParam.sigmaPT;
	double sPR = kalmanParam.sigmaPR;
	vEst.resize(6);
	XEst.resize(13);
	XMes.resize(13);
	XPred.resize(13);
	Q.resize(13,13);
	R.resize(13,13);
	J.resize(13,13);
	J.setIdentity();
	H.resize(7,13);
	H_0.resize(6,13);
	PEst.resize(13,13);
	Q.setIdentity();
	Q =sQT*sQT*Q;
	PEst.setIdentity();
	PEst = sPT*sPT*PEst;

	K.resize(13,13);

	K.setIdentity();
	for(int j = 0; j<13; j++)
		for(int i = 0; i<7; i++)
		{
			if(i<7 && j<7)
				{
				if(i==j)
					{
					H[i][j] = 1;
					}
				}
		}
	for(int j = 0; j<13; j++)
		for(int i = 0; i<6; i++)
		{
			if(i<6 && j>=7)
				{
				if(i==j-7)
					{
					H_0[i][j] = 1;
					}
				}
		}

	for(int j = 0; j<13; j++)
		for(int i = 0; i<13; i++)
		{
			if(i>=7 && j<7)
				{
				if(j==i-7)
					{
					//K[i][j] = 1;
					}
				}
			if(i<7 && j>=7)
				{
				if(i==j-7)
					{J[i][j] = 1;
					//H_0[i][j] = 1;
					//H[i][j] = 1;
					}
				}
			else if(j<7 && i<7)
				{
				Q[i][i] = sQR*sQR;
				if(i == j)
				{//H[i][j] = 1;
				//R[i][j] = 0.02;
				}
				if (j<3 && i<3){
						Q[i][i] = sQT*sQT;
						//R[i][j] = 0.001;
				}
				}
		}
	Q[0][0] = sQT*sQT;
	Q[1][1] = sQT*sQT;
	Q[2][2] = sQT*sQT;
	Q[10][10] = sQR*sQR;
	Q[11][11] = sQR*sQR;
	Q[12][12] = sQR*sQR;
	PEst[3][3] = sPR*sPR;
	PEst[4][4] = sPR*sPR;
	PEst[5][5] = sPR*sPR;
	PEst[6][6] = sPR*sPR;
	PEst[10][10] = sPR*sPR;
	PEst[11][11] = sPR*sPR;
	PEst[12][12] = sPR*sPR;

	cMoEst = cMo;
	cMoEst_0 = cMo;
	vEst[0] = 0;
	vEst[1] = 0;
	vEst[2] = 0;
	vEst[3] = 0;
	vEst[4] = 0;
	vEst[5] = 0;
	vMes = vEst;
	vpColVector poseQ(7);
	std::cout << " cMoo " << cMoEst << std::cout;
	convert(cMoEst,poseQ);
	XEst[0] = poseQ[0];
	XEst[1] = poseQ[1];
	XEst[2] = poseQ[2];
	XEst[3] = poseQ[3];
	XEst[4] = poseQ[4];
	XEst[5] = poseQ[5];
	XEst[6] = poseQ[6];
	std::cout << " cMooQuat " << cMoEst << std::cout;
	XEst[7] = vEst[0];
	XEst[8] = vEst[1];
	XEst[9] = vEst[2];
	XEst[10] = vEst[3];
	XEst[11] = vEst[4];
	XEst[12] = vEst[5];
	XMes = XEst;
}

void apKalmanFilterQuat::predictPose()
{
	vPred = vEst;
	double dt = 1;
	std::cout << " pred " << std::endl;
	computeJacobian(dt);
	//vPred = vpExponentialMap::inverse(cMoEst*cMoEst_0.inverse());
	vpMatrix Iv(6,6);
	Iv.setIdentity();
	vpMatrix I(13,13);
	I.setIdentity();

	XPred = XEst;
	XPred[0] += dt*XEst[7];
	XPred[1] += dt*XEst[8];
	XPred[2] += dt*XEst[9];
	XPred[3] += dt*( XEst[12]*XEst[4] - XEst[11]*XEst[5] + XEst[10]*XEst[6]);
	XPred[4] += dt*( -XEst[12]*XEst[3] + XEst[10]*XEst[5] + XEst[11]*XEst[6]);
	XPred[5] += dt*( XEst[11]*XEst[3] - XEst[10]*XEst[4] + XEst[12]*XEst[6]);
	XPred[6] += dt*( -XEst[10]*XEst[3] - XEst[11]*XEst[4] - XEst[12]*XEst[5]);
	std::cout << " XPred diff 0 " << XPred << std::endl;

	vpColVector wEst(3);
	wEst[0] = vEst[3];
	wEst[1] = vEst[4];
	wEst[2] = vEst[5];
	vpMatrix phi = computePhi(wEst);
	//XPred = phi*XEst;
	std::cout << " XPred diff 1 " << XPred << std::endl;

	//XPred = J*XEst;

	std::cout << " XPred diff 2 " << XPred << std::endl;
	//std::cout << " Phi " << phi << std::endl;
	std::cout << " J " << J << std::endl;

	std::cout << " XPred 0 " << XPred << std::endl;

	cMoPred = vpExponentialMap::direct(vPred).inverse() * cMoEst;
	//cMoPred_0 = vpExponentialMap::direct(vMes-vPred).inverse() * cMoEst;

	cMoPred_0 = cMoEst;
	std::cout << " Q " << K << std::endl;
	std::cout << " H " << H_0*K*H_0.transpose() << std::endl;

	cMoPred_0 = vpExponentialMap::direct((Iv-H_0*K*H_0.transpose())*vPred).inverse() * cMoEst;

	std::cout << " H0KH0 " << H_0*K*H_0.transpose() << std::endl;


	PPred = J*PEst*J.transpose() + Q;
	//PPred = phi*PEst*phi.transpose() + Q;
	std::cout << " H_0 " << H_0 << std::endl;
    cMoEst_0 = cMoEst;
}

void apKalmanFilterQuat::estimatePose(vpHomogeneousMatrix &cMoMes, vpMatrix &covMes)
{
vpColVector poseQuat(7);
convert(cMoMes, poseQuat);
XMes.resize(13);
vMes = vpExponentialMap::inverse((cMoMes*cMoEst.inverse()).inverse());
XMes[0] = poseQuat[0];
XMes[1] = poseQuat[1];
XMes[2] = poseQuat[2];
XMes[3] = poseQuat[3];
XMes[4] = poseQuat[4];
XMes[5] = poseQuat[5];
XMes[6] = poseQuat[6];
XMes[7] = vMes[0];
XMes[8] = vMes[1];
XMes[9] = vMes[2];
XMes[10] = vMes[3];
XMes[11] = vMes[4];
XMes[12] = vMes[5];
vpTranslationVector tr;
vpRotationMatrix R0;
vpThetaUVector r;
cMoMes.extract(tr);
cMoMes.extract(R0);
cMoInnov = cMoMes*(cMoPred.inverse());
R.resize(13,13);
for(int i = 0;i<3;i++)
	R[i][i] = 0.001;
for(int i = 3;i<7;i++)
	R[i][i] = 0.00001;

for(int i = 7;i<R.getRows();i++)
	for(int j = 7; j<R.getCols(); j++)
	{
//R[i][j] = covMes[i-7][j-7];
		if (i<10)
R[i][i] = 0.1;
		else R[i][i] = 0.01;
	}

//K = PPred*H.transpose()*((H*PPred*H.transpose() + R).pseudoInverse());
K = PPred*((PPred + R).pseudoInverse());

/*Kv[0][0] = 0.1;
Kv[1][1] = 0.1;
Kv[2][2] = 0.1;
Kv[3][3] = 0.1;
Kv[4][4] = 0.1;
Kv[5][5] = 0.1;*/

/*K.resize(13,13);
for (int i = 0; i<13; i++)
K[i][i] = 0.9;*/


vpMatrix I(13,13);
I.setIdentity();
vpMatrix Iv(6,6);
Iv.setIdentity();

std::cout << " K " << K << std::endl;
std::cout << " R " << R << std::endl;

//vEst = vMes;
//vEst = vPred + (Kv)*(vMes-vPred);
//vEst = vPred + (H_0)*K*(vMes-vPred);
//XEst = XPred + K*(poseQuat - H*XPred);
XEst = XPred + K*(XMes - XPred);
for (int i = 0; i<7; i++)
poseQuat[i] = XEst[i];
for (int i = 7; i<13; i++)
vEst[i-7] = XEst[i];
std::cout << " pose Quat " << poseQuat << std::endl;
//cMoEst = vpExponentialMap::direct(Kv*(vMes-vPred)).inverse()*cMoPred;
//cMoEst = vpExponentialMap::direct(Kv*vpExponentialMap::inverse(cMoInnov.inverse())).inverse()*cMoPred;
//cMoEst = vpExponentialMap::direct(H_0*K*vpExponentialMap::inverse(cMoInnov.inverse())).inverse()*cMoPred;
convert(poseQuat,cMoEst);
//cMoEst = cMoMes;
//PEst = (I-K*H)*PPred;
PEst = (I-K)*PPred;
//PvEst = (Iv-Kv)*PvPred;
}

void apKalmanFilterQuat::convert(vpHomogeneousMatrix &_cMo, vpColVector &poseQuat_)
{
	  poseQuat_.resize(7);
	  vpRotationMatrix _R;
	  vpTranslationVector _tr;
	  _cMo.extract(_R);
	  _cMo.extract(_tr);
	  vpThetaUVector tu(_R);
	  vpColVector u;
	  double theta;
	  tu.extract(theta, u);
	  theta *= 0.5;
	  double sinTheta_2 = sin(theta);
	  poseQuat_[0] = _tr[0];
	  poseQuat_[1] = _tr[1];
	  poseQuat_[2] = _tr[2];
	  poseQuat_[3] = u[0] * sinTheta_2;
	  poseQuat_[4] = u[1] * sinTheta_2;
	  poseQuat_[5] = u[2] * sinTheta_2;
	  poseQuat_[6] = cos(theta);
}


void apKalmanFilterQuat::convert(vpColVector &_poseQuat, vpHomogeneousMatrix &cMo_)
{
	  double a = _poseQuat[6];
	  double b = _poseQuat[3];
	  double c = _poseQuat[4];
	  double d = _poseQuat[5];
	  vpRotationMatrix R0;
	  R0[0][0] = a*a+b*b-c*c-d*d;
	  R0[0][1] = 2*b*c-2*a*d;
	  R0[0][2] = 2*a*c+2*b*d;

	  R0[1][0] = 2*a*d+2*b*c;
	  R0[1][1] = a*a-b*b+c*c-d*d;
	  R0[1][2] = 2*c*d-2*a*b;

	  R0[2][0] = 2*b*d-2*a*c;
	  R0[2][1] = 2*a*b+2*c*d;
	  R0[2][2] = a*a-b*b-c*c+d*d;

	  vpTranslationVector tr;
	  tr[0] = _poseQuat[0];
	  tr[1] = _poseQuat[1];
	  tr[2] = _poseQuat[2];

	  cMo_.buildFrom(tr,R0);
	/*RVraie[0][0] = vpMath::sqr(_poseQuat[0]) +vpMath::sqr(_poseQuat[1]) - vpMath::sqr(_poseQuat[2]) - vpMath::sqr(quatVraie[3]);
	RVraie[0][1] = 2*quatVraie[1]*quatVraie[2]-2*quatVraie[0]*quatVraie[3];
	RVraie[0][2] = 2*quatVraie[0]*quatVraie[2]+2*quatVraie[1]*quatVraie[3];
	RVraie[1][0] = 2*quatVraie[0]*quatVraie[3]+2*quatVraie[1]*quatVraie[2];
	RVraie[1][1] = vpMath::sqr(quatVraie[0]) -vpMath::sqr(quatVraie[1]) + vpMath::sqr(quatVraie[2]) - vpMath::sqr(quatVraie[3]);
	RVraie[1][2] = 2*quatVraie[2]*quatVraie[3]-2*quatVraie[0]*quatVraie[1];
	RVraie[2][0] = 2*quatVraie[1]*quatVraie[3]-2*quatVraie[0]*quatVraie[2];
	RVraie[2][1] = 2*quatVraie[0]*quatVraie[1]+2*quatVraie[2]*quatVraie[3];
	RVraie[2][2] = vpMath::sqr(quatVraie[0]) -vpMath::sqr(quatVraie[1]) - vpMath::sqr(quatVraie[2]) + vpMath::sqr(quatVraie[3]);*/
	  std::cout << "cMoest " << tr << " R " << R0 << std::endl;
}


void apKalmanFilterQuat::computeJacobian(double dt)
{
	J.resize(13,13);
	J.setIdentity();
	J[0][7] = dt;
	J[1][8] = dt;
	J[2][9] = dt;
	J[3][4] = dt*XEst[12]/2.0;
	J[3][5] = -dt*XEst[11]/2.0;
	J[3][6] = dt*XEst[10]/2.0;
	J[3][10] = dt*XEst[6]/2.0;
	J[3][11] = -dt*XEst[5]/2.0;
	J[3][12] = dt*XEst[4]/2.0;
	J[4][3] = -dt*XEst[12]/2.0;
	J[4][5] = dt*XEst[10]/2.0;
	J[4][6] = dt*XEst[11]/2.0;
	J[4][10] = dt*XEst[5]/2.0;
	J[4][11] = dt*XEst[6]/2.0;
	J[4][12] = -dt*XEst[3]/2.0;
	J[5][3] = dt*XEst[11]/2.0;
	J[5][4] = -dt*XEst[10]/2.0;
	J[5][6] = dt*XEst[12]/2.0;
	J[5][10] = -dt*XEst[4]/2.0;
	J[5][11] = dt*XEst[3]/2.0;
	J[5][12] = dt*XEst[6]/2.0;
	J[6][3] = -dt*XEst[10]/2.0;
	J[6][4] = -dt*XEst[11]/2.0;
	J[6][5] = -dt*XEst[12]/2.0;
	J[6][10] = -dt*XEst[3]/2.0;
	J[6][11] = -dt*XEst[4]/2.0;
	J[6][12] = -dt*XEst[5]/2.0;
}

vpMatrix apKalmanFilterQuat::getTheta(vpColVector &w)
{
	vpMatrix theta;
	theta.resize(4,4);
	theta[0][0] = 0;
	theta[0][1] = w[2];
	theta[0][2] = -w[1];
	theta[0][3] = w[0];
	theta[1][0] = -w[2];
	theta[1][1] = 0;
	theta[1][2] = w[0];
	theta[1][3] = w[1];
	theta[2][0] = w[1];
	theta[2][1] = -w[0];
	theta[2][2] = 0;
	theta[2][3] = w[2];
	theta[3][0] = -w[0];
	theta[3][1] = -w[1];
	theta[3][2] = -w[2];
	theta[3][3] = 0;
	return theta;
}

vpMatrix apKalmanFilterQuat::computeExp(vpMatrix &Theta, vpColVector &w)
{
	vpMatrix exp(4,4);
	vpMatrix I(4,4);
	I.setIdentity();
	double norm = sqrt(w.sumSquare());
	if(norm > 0)
	exp = cos(0.5*norm)*I + (1/norm)*sin(0.5*norm)*Theta;
	else exp = I;
	return exp;
}

vpMatrix apKalmanFilterQuat::computePhi(vpColVector &w)
{
	vpMatrix phi(13,13);
	phi.setIdentity();
	vpMatrix theta = getTheta(w);
	vpMatrix exp = computeExp(theta,w);
	phi[0][7] = 1;
	phi[1][8] = 1;
	phi[2][9] = 1;
	for (int i = 3; i < 7; i++)
		for (int j = 3; j < 7; j++)
			phi[i][j] = exp[i-3][j-3];

	return phi;
}
