/*
 * apKalmanFilterSE32.cpp
 *
 *  Created on: May 28, 2013
 *      Author: agpetit
 */

#include "apKalmanFilterSE32.h"

apKalmanFilterSE32::apKalmanFilterSE32() {
	// TODO Auto-generated constructor stub

}

apKalmanFilterSE32::~apKalmanFilterSE32() {
	// TODO Auto-generated destructor stub
}

void apKalmanFilterSE32::initFilter(vpHomogeneousMatrix &cMo, apKalmanParameters &kalmanParam)
{

	double sQT = kalmanParam.sigmaQT;
	double sQR = kalmanParam.sigmaQR;
	double sPT = kalmanParam.sigmaPT;
	double sPR = kalmanParam.sigmaPR;
	vEst.resize(6);
	Q.resize(12,12);
	R.resize(6,6);
	J.resize(18,18);
	Jn.resize(18,6);
	H.resize(6,12);
	H_0.resize(6,18);
	PEst.resize(18,18);
	J.setIdentity();
	Q.setIdentity();
	L.resize(6,12);
	Q =sQT*sQT*Q;
	PEst.setIdentity();
	PEst = sPT*sPT*PEst;
	XEst.resize(18);
    XPred.resize(18);
    XMes.resize(18);
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
					{//J[i][j] = 1;
					//H_0[i][j] = 1;
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

		for(int i = 0; i<6; i++)
			H_0[i][i+12] = 1;

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
	XEst[0] = cMoEst[0][0];
	XEst[1] = cMoEst[1][0];
	XEst[2] = cMoEst[2][0];
	XEst[3] = cMoEst[0][1];
	XEst[4] = cMoEst[1][1];
	XEst[5] = cMoEst[2][1];
	XEst[6] = cMoEst[0][2];
	XEst[7] = cMoEst[1][2];
	XEst[8] = cMoEst[2][2];
	XEst[9] = cMoEst[0][3];
	XEst[10] = cMoEst[1][3];
	XEst[11] = cMoEst[2][3];
}

void apKalmanFilterSE32::predictPose()
{
	vPred = vEst;
	//vPred = vpExponentialMap::inverse(cMoEst*cMoEst_0.inverse());
	vpMatrix Iv(6,6);
	Iv.setIdentity();
	vpMatrix I(12,12);
	I.setIdentity();
	double dt =1;
	computeJacobian(dt);
	cMoPred = vpExponentialMap::direct(vPred).inverse() * cMoEst;
	XPred = F*XEst;
	std::cout << " Xpred " << XPred << std::endl;
	//cMoPred_0 = vpExponentialMap::direct((Iv-Kv)*vPred).inverse() * cMoEst;
	//std::cout << " Q " << H_0*K << std::endl;

	//cMoPred_0 = vpExponentialMap::direct((Iv-H_0*K)*vPred).inverse() * cMoEst;
	//cMoPred_0 = vpExponentialMap::direct(vMes-vPred).inverse() * cMoEst;
	cMoPred_0 = cMoEst;
	PPred = J*PEst*J.transpose() + Jn*Qv*Jn.transpose();
    PvPred = PvEst + Qv;
    cMoEst_0 = cMoEst;

	XPred[0] = cMoPred[0][0];
	XPred[1] = cMoPred[1][0];
	XPred[2] = cMoPred[2][0];
	XPred[3] = cMoPred[0][1];
	XPred[4] = cMoPred[1][1];
	XPred[5] = cMoPred[2][1];
	XPred[6] = cMoPred[0][2];
	XPred[7] = cMoPred[1][2];
	XPred[8] = cMoPred[2][2];
	XPred[9] = cMoPred[0][3];
	XPred[10] = cMoPred[1][3];
	XPred[11] = cMoPred[2][3];
	XPred[12] = vPred[0];
	XPred[13] = vPred[1];
	XPred[14] = vPred[2];
	XPred[15] = vPred[3];
	XPred[16] = vPred[4];
	XPred[17] = vPred[5];

	std::cout << " Xpred 0 " << XPred << std::endl;

}

void apKalmanFilterSE32::estimatePose(vpHomogeneousMatrix &cMoMes, vpMatrix &covMes)
{

cMoInnov = cMoMes*(cMoPred.inverse());
//std::cout << " H_0 " << H_0 << std::endl;
//R = covMes;
K = PPred*H_0.transpose()*((H_0*PPred*H_0.transpose() + covMes).pseudoInverse());
//Kv = PvPred*((PvPred + R).pseudoInverse());
//Kv = PvPred*((PvPred + covMes).pseudoInverse());

/*Kv[0][0] = 0.3;
Kv[1][1] = 0.3;
Kv[2][2] = 0.3;
Kv[3][3] = 0.3;
Kv[4][4] = 0.3;
Kv[5][5] = 0.3;*/
vpMatrix I(18,18);
I.setIdentity();
vpMatrix Iv(6,6);
Iv.setIdentity();
//cMoEst = vpExponentialMap::direct(K*vpExponentialMap::inverse(cMoInnov))*cMoPred;
//cMoEst = vpExponentialMap::direct(Kv*vpExponentialMap::inverse(cMoInnov.inverse())).inverse()*cMoPred;
//vEst = vpExponentialMap::inverse(cMoMes*cMoEst.inverse());
vMes = vpExponentialMap::inverse((cMoMes*cMoEst.inverse()).inverse());

XMes[0] = cMoMes[0][0];
XMes[1] = cMoMes[1][0];
XMes[2] = cMoMes[2][0];
XMes[3] = cMoMes[0][1];
XMes[4] = cMoMes[1][1];
XMes[5] = cMoMes[2][1];
XMes[6] = cMoMes[0][2];
XMes[7] = cMoMes[1][2];
XMes[8] = cMoMes[2][2];
XMes[9] = cMoMes[0][3];
XMes[10] = cMoMes[1][3];
XMes[11] = cMoMes[2][3];
XMes[12] = vMes[0];
XMes[13] = vMes[1];
XMes[14] = vMes[2];
XMes[15] = vMes[3];
XMes[16] = vMes[4];
XMes[17] = vMes[5];

//cMoEst = cMoMes;
//std::cout << " vpred " << vPred << " vmes " << vMes <<std::endl;
//vEst = vPred + K*(vpExponentialMap::inverse(cMoInnov.inverse()));
//vEst = vPred + (Kv)*(vpExponentialMap::inverse(cMoInnov.inverse()));

//std::cout << " Kv " << (H_0)*K << std::endl;
//std::cout << " K " << K << std::endl;
//vEst = vMes;
//vEst = vPred + (Kv)*(vMes-vPred);
//XEst = XPred + (K)*(XMes-XPred);
XEst = XPred + (K)*(vMes-vPred);
//vEst = vPred + (H_0)*K*(vMes-vPred);
//cMoEst = vpExponentialMap::direct(Kv*(vMes-vPred)).inverse()*cMoPred;
//cMoEst = vpExponentialMap::direct(Kv*vpExponentialMap::inverse(cMoInnov.inverse())).inverse()*cMoPred;
//cMoEst = vpExponentialMap::direct(H_0*K*vpExponentialMap::inverse(cMoInnov.inverse())).inverse()*cMoPred;
//cMoEst = cMoMes;
cMoEst[0][0] = XEst[0];
cMoEst[1][0] = XEst[1];
cMoEst[2][0] = XEst[2];
cMoEst[0][1] = XEst[3];
cMoEst[1][1] = XEst[4];
cMoEst[2][1] = XEst[5];
cMoEst[0][2] = XEst[6];
cMoEst[1][2] = XEst[7];
cMoEst[2][2] = XEst[8];
cMoEst[0][3] = XEst[9];
cMoEst[1][3] = XEst[10];
cMoEst[2][3] = XEst[11];
vEst[0] = XEst[12];
vEst[1] = XEst[13];
vEst[2] = XEst[14];
vEst[3] = XEst[15];
vEst[4] = XEst[16];
vEst[5] = XEst[17];

//PEst = (I-K*H)*PPred;
PEst = (I-K*H_0)*PPred;
//PvEst = (Iv-Kv)*PvPred;
}

void apKalmanFilterSE32::computeJacobian(double dt)
{
vpMatrix Id(3,3);
Id.setIdentity();
vpThetaUVector wu;
wu[0] = -vEst[3];
wu[1] = -vEst[4];
wu[2] = -vEst[5];
vpMatrix exp = vpExponentialMap::direct(vEst).inverse();
vpMatrix expRot(3,3);
for (int i=0;i<3;i++)
	for(int j=0;j<3;j++)
		expRot[i][j] = exp[i][j];

vpMatrix IoexpR(9,9),Iodiffsinc(3,9),IoskewM(9,9),Iodiffcosc(3,9),IoskewM2(9,9),IodiffR(9,27),IoXv(27,3),Ioexp(12,12);
vpMatrix::kron(Id,expRot,IoexpR);
vpMatrix::kron(Id,exp,Ioexp);
J.resize(18,18);
std::cout << "IoexpR " << IoexpR << std::endl;
J = vpMatrix::insert(J,IoexpR,0,0);
J = vpMatrix::insert(J,Id,9,9);
J = vpMatrix::insert(J,Id,9,12);
J = vpMatrix::insert(J,Id,12,12);
J = vpMatrix::insert(J,Id,15,15);
F = J;
std::cout << "Jacobian0 " << J << std::endl;


vpMatrix diffR;
vpMatrix diffSkew(3,9);
diffSkew[0][5] = 1;
diffSkew[0][7] = -1;
diffSkew[1][2] = -1;
diffSkew[1][6] = 1;
diffSkew[2][1] = 1;
diffSkew[2][3] = -1;
//diffSkew.transpose(diffSkew);
double w;
vpColVector u;
wu.extract(w,u);
vpMatrix skewM = vpColVector::skew(wu);
vpRowVector diffsinc, diffcosc;
double sinc,cosc,diffsincw,diffcoscw;
if(w!=0)
{
sinc = (sin(w)/w);
diffsincw = (w*cos(w) - sin(w))/(w*w);
cosc = (1-cos(w))/(w*w);
diffcoscw = (sin(w)*w - 2 + 2*cos(w))/(w*w*w);
}
else
{
sinc = 1;
diffsincw = 0;
cosc = 1;
diffcoscw =0;
}

diffsinc = diffsincw*(u.transpose());
diffcosc = diffcoscw*(u.transpose());

vpMatrix::kron(Id,diffsinc,Iodiffsinc);
vpMatrix::kron(Id,skewM,IoskewM);

//std::cout << "IoskewM " << IoskewM << std::endl;
vpMatrix diffR1 = Iodiffsinc*IoskewM + sinc*diffSkew;
//std::cout << "diffR1 " << diffR1 << std::endl;

vpMatrix diffSkewM2 = diffSkew*IoskewM + skewM*diffSkew;
//std::cout << "diffR2 " << diffSkewM2 << std::endl;
vpMatrix::kron(Id,diffcosc,Iodiffcosc);
vpMatrix::kron(Id,skewM*skewM,IoskewM2);
vpMatrix diffR2 = Iodiffcosc*IoskewM2 + cosc*diffSkewM2;
diffR = diffR1 + diffR2;
//std::cout << "diffR " << diffR << std::endl;

//std::cout << "diffR " << diffR << std::endl;
//vpMatrix::kron(Id,diffR,IodiffR);
vpMatrix diffRw0(3,3),diffRw1(3,3),diffRw2(3,3);
vpMatrix IodiffRw0(9,9),IodiffRw1(9,9),IodiffRw2(9,9);
for (int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	{
		/*diffRw0[i][j] = diffR[i][j];
		diffRw1[i][j] = diffR[i][j+3];
		diffRw2[i][j] = diffR[i][j+6];*/
		diffRw0[i][j] = diffSkew[i][j];
		diffRw1[i][j] = diffSkew[i][j+3];
		diffRw2[i][j] = diffSkew[i][j+6];
	}
vpMatrix::kron(Id,diffRw0,IodiffRw0);
vpMatrix::kron(Id,diffRw1,IodiffRw1);
vpMatrix::kron(Id,diffRw2,IodiffRw2);
vpMatrix buff(18,9),diffKron(27,9);
vpMatrix::stackMatrices(IodiffRw0.transpose(),IodiffRw1.transpose(),buff);
vpMatrix::stackMatrices(buff,IodiffRw2.transpose(),diffKron);
//std::cout << "diffKron " << diffKron << std::endl;
vpColVector Xv = XEst;
Xv.resize(9,false);
vpMatrix::kron(Id,Xv,IoXv);
vpMatrix diffw = diffKron.transpose()*IoXv;
J = vpMatrix::insert(J,diffw,0,15);
//std::cout << "Jacobian " << J << std::endl;
for (int i=0;i<18;i++)
	for(int j=0;j<6;j++)
	Jn[i][j]=J[i][j+12];
}
