/*
 * apKalmanFilterSE33.cpp
 *
 *  Created on: May 29, 2013
 *      Author: agpetit
 */

#include "apKalmanFilterSE33.h"

apKalmanFilterSE33::apKalmanFilterSE33() {
	// TODO Auto-generated constructor stub

}

apKalmanFilterSE33::~apKalmanFilterSE33() {
	// TODO Auto-generated destructor stub
}

void apKalmanFilterSE33::initFilter(vpHomogeneousMatrix &cMo, apKalmanParameters &kalmanParam)
{

	double sQT = kalmanParam.sigmaQT;
	double sQR = kalmanParam.sigmaQR;
	double sPT = kalmanParam.sigmaPT;
	double sPR = kalmanParam.sigmaPR;
	vEst.resize(6);
	Q.resize(12,12);
	R.resize(6,6);
	J.resize(22,22);
	Jn.resize(22,6);
	H.resize(6,12);
	H_0.resize(6,22);
	PEst.resize(22,22);
	J.setIdentity();
	Q.setIdentity();
	L.resize(6,12);
	Q =sQT*sQT*Q;
	PEst.setIdentity();
	PEst = sPT*sPT*PEst;
	XEst.resize(22);
    XPred.resize(22);
    XMes.resize(22);
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
			H_0[i][i+16] = 1;

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

void apKalmanFilterSE33::predictPose()
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
	std::cout << " Xpred0 " << XPred << std::endl;

	XPred = J*XEst;

	std::cout << " Xpred1 " << XPred << std::endl;

	//cMoPred_0 = vpExponentialMap::direct((Iv-Kv)*vPred).inverse() * cMoEst;
	//std::cout << " Q " << H_0*K << std::endl;

	//cMoPred_0 = vpExponentialMap::direct((Iv-H_0*K)*vPred).inverse() * cMoEst;
	//cMoPred_0 = vpExponentialMap::direct(vMes-vPred).inverse() * cMoEst;
	cMoPred_0 = cMoEst;
	PPred = J*PEst*J.transpose() + Jn*Qv*Jn.transpose();

	//std::cout << "Jacobiann " <<  Jn*Qv*Jn.transpose() << std::endl;

    PvPred = PvEst + Qv;
    cMoEst_0 = cMoEst;

	XPred[0] = cMoPred[0][0];
	XPred[1] = cMoPred[1][0];
	XPred[2] = cMoPred[2][0];
	XPred[3] = cMoPred[3][0];
	XPred[4] = cMoPred[0][1];
	XPred[5] = cMoPred[1][1];
	XPred[6] = cMoPred[2][1];
	XPred[7] = cMoPred[3][1];
	XPred[8] = cMoPred[0][2];
	XPred[9] = cMoPred[1][2];
	XPred[10] = cMoPred[2][2];
	XPred[11] = cMoPred[3][2];
	XPred[12] = cMoPred[0][3];
	XPred[13] = cMoPred[1][3];
	XPred[14] = cMoPred[2][3];
	XPred[15] = cMoPred[3][3];
	XPred[16] = vPred[0];
	XPred[17] = vPred[1];
	XPred[18] = vPred[2];
	XPred[19] = vPred[3];
	XPred[20] = vPred[4];
	XPred[21] = vPred[5];

	std::cout << " Xpred2 " << XPred << std::endl;

}

void apKalmanFilterSE33::estimatePose(vpHomogeneousMatrix &cMoMes, vpMatrix &covMes)
{

cMoInnov = cMoMes*(cMoPred.inverse());
std::cout << " H_0 " << H_0 << std::endl;
//R = covMes;
K = PPred*H_0.transpose()*((H_0*PPred*H_0.transpose() + Rv).pseudoInverse());
//Kv = PvPred*((PvPred + R).pseudoInverse());
//Kv = PvPred*((PvPred + covMes).pseudoInverse());

/*Kv[0][0] = 0.3;
Kv[1][1] = 0.3;
Kv[2][2] = 0.3;
Kv[3][3] = 0.3;
Kv[4][4] = 0.3;
Kv[5][5] = 0.3;*/
vpMatrix I(22,22);
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
XMes[3] = cMoMes[3][0];
XMes[4] = cMoMes[0][1];
XMes[5] = cMoMes[1][1];
XMes[6] = cMoMes[2][1];
XMes[7] = cMoMes[3][1];
XMes[8] = cMoMes[0][2];
XMes[9] = cMoMes[1][2];
XMes[10] = cMoMes[2][2];
XMes[11] = cMoMes[3][2];
XMes[12] = cMoMes[0][3];
XMes[13] = cMoMes[1][3];
XMes[14] = cMoMes[2][3];
XMes[15] = cMoMes[3][3];
XMes[16] = vMes[0];
XMes[17] = vMes[1];
XMes[18] = vMes[2];
XMes[19] = vMes[3];
XMes[20] = vMes[4];
XMes[21] = vMes[5];

//cMoEst = cMoMes;
//std::cout << " vpred " << vPred << " vmes " << vMes <<std::endl;
//vEst = vPred + K*(vpExponentialMap::inverse(cMoInnov.inverse()));
//vEst = vPred + (Kv)*(vpExponentialMap::inverse(cMoInnov.inverse()));

//std::cout << " Kv " << (H_0)*K << std::endl;
std::cout << " K " << K << std::endl;
//vEst = vMes;
//vEst = vPred + (Kv)*(vMes-vPred);
/*K.resize(22,22);
K.setIdentity();
K*=0.5;
XEst = XPred + (K)*(XMes-XPred);*/
XEst = XPred + (K)*(vMes-vPred);
//vEst = vPred + (H_0)*K*(vMes-vPred);
//XEst = XMes;
//cMoEst = vpExponentialMap::direct(Kv*(vMes-vPred)).inverse()*cMoPred;
//cMoEst = vpExponentialMap::direct(Kv*vpExponentialMap::inverse(cMoInnov.inverse())).inverse()*cMoPred;
//cMoEst = vpExponentialMap::direct(H_0*K*vpExponentialMap::inverse(cMoInnov.inverse())).inverse()*cMoPred;
//cMoEst = cMoMes;
cMoEst[0][0] = XEst[0];
cMoEst[1][0] = XEst[1];
cMoEst[2][0] = XEst[2];
cMoEst[3][0] = XEst[3];
cMoEst[0][1] = XEst[4];
cMoEst[1][1] = XEst[5];
cMoEst[2][1] = XEst[6];
cMoEst[3][1] = XEst[7];
cMoEst[0][2] = XEst[8];
cMoEst[1][2] = XEst[9];
cMoEst[2][2] = XEst[10];
cMoEst[3][2] = XEst[11];
cMoEst[0][3] = XEst[12];
cMoEst[1][3] = XEst[13];
cMoEst[2][3] = XEst[14];
cMoEst[3][3] = XEst[15];
vEst[0] = XEst[16];
vEst[1] = XEst[17];
vEst[2] = XEst[18];
vEst[3] = XEst[19];
vEst[4] = XEst[20];
vEst[5] = XEst[21];

//PEst = (I-K)*PPred;
PEst = (I-K*H_0)*PPred;
//PvEst = (Iv-Kv)*PvPred;
}

void apKalmanFilterSE33::computeJacobian(double dt)
{
vpMatrix Id(3,3);
Id.setIdentity();
vpMatrix Id4(4,4);
Id4.setIdentity();
vpThetaUVector wu;
wu[0] = vEst[3];
wu[1] = vEst[4];
wu[2] = vEst[5];
vpMatrix exp = vpExponentialMap::direct(vEst).inverse();
vpMatrix expRot(3,3);
for (int i=0;i<3;i++)
	for(int j=0;j<3;j++)
		expRot[i][j] = exp[i][j];

vpMatrix IoexpR(9,9),Ioexp(16,16);
vpMatrix::kron(Id,expRot,IoexpR);
vpMatrix::kron(Id4,exp,Ioexp);
J.resize(22,22);
std::cout << "IoexpR " << IoexpR << std::endl;
J = vpMatrix::insert(J,Ioexp,0,0);
J = vpMatrix::insert(J,Id,16,16);
J = vpMatrix::insert(J,Id,19,19);
F = J;
//std::cout << "Jacobian0 " << J << std::endl;


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

vpMatrix IoXv(48,3);
vpColVector Xv = XEst;
Xv.resize(16,false);
vpMatrix::kron(Id,Xv,IoXv);

vpRowVector diffsinc, diffcosc,diffsind;
vpMatrix Iodiffsinc(3,9),Iodiffcosc(3,9), Iodiffsind(3,9);
double sinc,cosc,sind,diffsincw,diffcoscw,diffsindw;
if(w!=0)
{
sinc = (sin(w)/w);
diffsincw = (w*cos(w) - sin(w))/(w*w);
cosc = (1-cos(w))/(w*w);
diffcoscw = (sin(w)*w - 2 + 2*cos(w))/(w*w*w);
sind = (1-sinc)/(w*w);
diffsindw = (sin(w)-w*cos(w) - (1-sinc)*2*w)/(w*w*w*w);
}
else
{
sinc = 1;
diffsincw = 0;
cosc = 1;
diffcoscw =0;
sind = 1;
diffsindw = 0;
}

diffsinc = diffsincw*(u.transpose());
diffcosc = diffcoscw*(u.transpose());
diffsind = diffsindw*(u.transpose());
vpMatrix::kron(Id,diffsinc,Iodiffsinc);
vpMatrix::kron(Id,diffcosc,Iodiffcosc);
vpMatrix::kron(Id,diffsind,Iodiffsind);


vpMatrix V(3,3);
V = Id + cosc*skewM + sind*skewM*skewM;
vpColVector v0(3),v1(3),v2(3);
for (int i=0;i<3;i++)
{
v0[i] = V[i][0];
v1[i] = V[i][1];
v2[i] = V[i][2];
}
vpMatrix diffIoexpV(16,48);
vpMatrix diffIoexpV0(16,16);
vpMatrix diffIoexpV1(16,16);
vpMatrix diffIoexpV2(16,16);
vpMatrix diffV0(4,4),diffV1(4,4),diffV2(4,4);
diffV0 = vpMatrix::insert(diffV0,v0,0,3);
diffV1 = vpMatrix::insert(diffV1,v1,0,3);
diffV2 = vpMatrix::insert(diffV2,v2,0,3);

std::cout << " diffV0 " << diffV0 << std::endl;

vpMatrix::kron(Id4,diffV0,diffIoexpV0);
vpMatrix::kron(Id4,diffV1,diffIoexpV1);
vpMatrix::kron(Id4,diffV2,diffIoexpV2);

vpMatrix diffVt0 = diffIoexpV0*Xv;
vpMatrix diffVt1 = diffIoexpV1*Xv;
vpMatrix diffVt2 = diffIoexpV2*Xv;
vpMatrix bufft(16,2);

vpMatrix buffV(16,32);

vpMatrix diffVT(16,3);
vpMatrix::juxtaposeMatrices(diffVt0,diffVt1,bufft);
vpMatrix::juxtaposeMatrices(bufft,diffVt2,diffVT);

std::cout << " diffVT " << diffVT << std::endl;

vpMatrix::juxtaposeMatrices(diffIoexpV0,diffIoexpV1,buffV);
vpMatrix::juxtaposeMatrices(buffV,diffIoexpV2,diffIoexpV);


vpMatrix diffV = diffIoexpV*IoXv;

std::cout << " diffV " << diffV << std::endl;

vpMatrix IoskewM(9,9);
vpMatrix::kron(Id,skewM,IoskewM);
vpMatrix IoskewM2(9,9);
vpMatrix::kron(Id,skewM*skewM,IoskewM2);
vpMatrix diffSkewM2 = diffSkew*IoskewM + skewM*diffSkew;

vpMatrix diffT;
vpMatrix diffT1 = Iodiffcosc*IoskewM + cosc*diffSkew;
vpMatrix diffT2 = Iodiffsind*IoskewM2 + sind*diffSkewM2;
diffT = diffT1 + diffT2;

vpMatrix IoXt(9,3);
vpColVector Xt(3);
Xt[0] = vEst[0];
Xt[1] = vEst[1];
Xt[2] = vEst[2];
vpMatrix::kron(Id,Xt,IoXt);
vpMatrix difft = diffT*IoXt;

vpColVector difft0(3),difft1(3),difft2(3);
for (int i=0;i<3;i++)
{
difft0[i] = difft[i][0];
difft1[i] = difft[i][1];
difft2[i] = difft[i][2];
}

vpMatrix diffR1 = Iodiffsinc*IoskewM + sinc*diffSkew;
vpMatrix diffR2 = Iodiffcosc*IoskewM2 + cosc*diffSkewM2;
diffR = diffR1 + diffR2;
//std::cout << "diffR " << diffR << std::endl;

//std::cout << "diffR " << diffR << std::endl;
//vpMatrix::kron(Id,diffR,IodiffR);

vpMatrix diffRw0(3,3),diffRw1(3,3),diffRw2(3,3);
for (int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	{
		diffRw0[i][j] = diffR[i][j];
		diffRw1[i][j] = diffR[i][j+3];
		diffRw2[i][j] = diffR[i][j+6];
		/*diffRw0[i][j] = diffSkew[i][j];
		diffRw1[i][j] = diffSkew[i][j+3];
		diffRw2[i][j] = diffSkew[i][j+6];*/
	}

vpMatrix diffw0(4,4),diffw1(4,4),diffw2(4,4);
diffw0 = vpMatrix::insert(diffw0,diffRw0,0,0);
diffw0 = vpMatrix::insert(diffw0,difft0,0,3);
diffw1 = vpMatrix::insert(diffw1,diffRw1,0,0);
diffw1 = vpMatrix::insert(diffw1,difft1,0,3);
diffw2 = vpMatrix::insert(diffw2,diffRw2,0,0);
diffw2 = vpMatrix::insert(diffw2,difft2,0,3);

vpMatrix Iodiffw0(16,16),Iodiffw1(16,16),Iodiffw2(16,16);

vpMatrix::kron(Id4,diffw0,Iodiffw0);
vpMatrix::kron(Id4,diffw1,Iodiffw1);
vpMatrix::kron(Id4,diffw2,Iodiffw2);

vpMatrix buff(16,32),diffKron(16,48);
vpMatrix::juxtaposeMatrices(Iodiffw0,Iodiffw1,buff);
vpMatrix::juxtaposeMatrices(buff,Iodiffw2,diffKron);
//std::cout << "diffKron " << diffKron << std::endl;

vpMatrix diffw = diffKron*IoXv;
diffw.resize(16,3);
diffV.resize(16,3);
J = vpMatrix::insert(J,diffV,0,16);
J = vpMatrix::insert(J,diffw,0,19);

//std::cout << "diffw " << diffw <<std::endl;
Jn.resize(22,6);
for (int i=0;i<22;i++)
	for(int j=0;j<6;j++)
	Jn[i][j]=J[i][j+16];

std::cout << "Jacobiann " << J << std::endl;

}
