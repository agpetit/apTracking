#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPoint.h>
#include <visp/vpTime.h>
#include <visp/vpRGBa.h>
#include <visp/vpIoTools.h>
#include <visp/vpFeatureDisplay.h>
#include "vpAROgre.h"
#include <cv.h>
#include <math.h>
#include "apViewGeneration.h"
#include "apImageFilter.h"
#define INF 1E6


using namespace std ;
using namespace cv;





/*!
  Initialize the memory space requested for vpFeatureLuminance visual feature.
*/
void
apViewGeneration::init()
{
EdgeOrientMaps.resize(1);
DTTemp.resize(1);
argDTTemp.resize(1);
mergeTemp.resize(1);
Ipyramid.resize(0);
//argDTTempNbhd.resize(1);
}


void
apViewGeneration::init(int s_Rx, int s_Ry, int s_Rz, double s_Z)
{
init() ;
sample_Rx=s_Rx;
sample_Ry=s_Ry;
sample_Rz=s_Rz;
sample_Z=s_Z;
}

void
apViewGeneration::initSph(int s_Rho, int s_Theta, int s_Phi)
{
sample_Rho=s_Rho;
sample_Theta=s_Theta;
sample_Phi=s_Phi;
}

void
apViewGeneration::initSphNbhd(int s_Rho, int s_Theta, int s_Phi, int nb_Theta, int nb_Phi)
{
sample_Rho=s_Rho;
sample_Theta=s_Theta;
sample_Phi=s_Phi;
nbhd_Theta = nb_Theta;
nbhd_Phi = nb_Phi;
snbhd_Theta = (int)s_Theta/nb_Theta;
snbhd_Phi =  (int)s_Phi/nb_Phi;
}

/*! 
  Default constructor that build a visual feature.
*/
apViewGeneration::apViewGeneration()
{
    init();
}

/*! 
  Default destructor.
*/
apViewGeneration::~apViewGeneration()
{
vpImage<unsigned char> *Ior;
vpImage<double> *IDT ;
vpImage<vpRGBa> *IargDT;

	 for (unsigned int i = 0; i < EdgeOrientMaps.size(); i += 1){
	        Ior = EdgeOrientMaps[i];
	        IDT = DTTemp[i];
	        IargDT = argDTTemp[i];
	        if (Ior!=NULL) delete Ior;
	        if (IDT!=NULL) delete IDT;
	        if (IargDT!=NULL) delete IargDT;
	        Ior = NULL;
	        IDT = NULL;
	        IargDT = NULL;
}
	  EdgeOrientMaps.resize(0);
	  DTTemp.resize(0);
	  argDTTemp.resize(0);
}

void apViewGeneration::createViews(vpAROgre &ogre, vpPoseVector PoseInit, std::string opath)
{
vpPoseVector PosIn;
vpHomogeneousMatrix cMo;
vpImage<unsigned char> Ior0;
cMo.buildFrom(PoseInit);
for (int k=0;k<sample_Z;k++){
for (int l=0;l<sample_Rx;l++){
for (int m=0;m<sample_Ry;m++){
for (int n=0;n<sample_Rz;n++){
int row=n+sample_Ry*m+(sample_Rx*sample_Rx)*l+(sample_Z*sample_Z*sample_Z)*k;
//vpImage<unsigned char> *Ior;// = new vpImage<unsigned char>;
PosIn[0]=PoseInit[0];
PosIn[1]=PoseInit[1];
PosIn[2]=PoseInit[2]-(0.2/2)+k*(0.2/(sample_Z-1));
PosIn[3]=PoseInit[3]-(M_PI/16)+l*(2*(M_PI/16)/(sample_Rx-1));
PosIn[4]=PoseInit[4]-(M_PI/16)+m*(2*(M_PI/16)/(sample_Ry-1));
PosIn[5]=PoseInit[5]-(M_PI/16)+n*(2*(M_PI/16)/(sample_Rz-1));
cMo.buildFrom(PosIn);
vpImage<unsigned char> *I00 = new vpImage<unsigned char>;
I00->resize(480,640);
ogre.updateRTTV(*I00,&cMo);
EdgeOrientMaps.push_back(I00);
//delete Ior;
Ior0=*I00;
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), row);
std::string filename(buf);
vpImageIo::writePPM(Ior0, filename);
		}
		}
		}
		}
		//}
	//}
}

void apViewGeneration::createViewsDT(vpAROgre &ogre, vpPoseVector PoseInit, std::string opath)
{
vpPoseVector PosIn;
vpHomogeneousMatrix cMo;
vpImage<unsigned char> Ior0;
vpImage<vpRGBa> *imArg;
cMo.buildFrom(PoseInit);
for (int k=0;k<sample_Z;k++){
for (int l=0;l<sample_Rx;l++){
for (int m=0;m<sample_Ry;m++){
for (int n=0;n<sample_Rz;n++){
int row=n+sample_Ry*m+(sample_Rx*sample_Rx)*l+(sample_Z*sample_Z*sample_Z)*k;
PosIn[0]=PoseInit[0];
PosIn[1]=PoseInit[1];
//PosIn[2]=PoseInit[2]-(0.2/2)+k*(0.2/(sample_Z-1));
PosIn[2]=PoseInit[2];
PosIn[3]=PoseInit[3]-M_PI/10-(M_PI/2)+l*(2*(M_PI/2)/(sample_Rx-1));
PosIn[4]=PoseInit[4]+M_PI/10-(M_PI/2)+m*(2*(M_PI/2)/(sample_Ry-1));
PosIn[5]=PoseInit[5]-M_PI/3-(M_PI/2)+n*(2*(M_PI/2)/(sample_Rz-1));
cMo.buildFrom(PosIn);
vpImage<unsigned char> *I00 = new vpImage<unsigned char>;;
I00->resize(480,640);
ogre.updateRTTV(*I00,&cMo);
imArg=dt0(I00);
// take square roots
for (int y = 0; y < I00->getHeight(); y++) {
  for (int x = 0; x < I00->getWidth(); x++) {
	  (*imArg)[y][x].A = (*I00)[y][x];
	  //cout << (double)(*imArg)[y][x].A << endl;
  }
}
argDTTemp.push_back(imArg);
//delete Ior;
Ior0=*I00;
delete I00;
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), row);
std::string filename(buf);
vpImageIo::writePPM(Ior0, filename);
		}
		}
		}
		}
		//}
	//}
}

void apViewGeneration::createViewsSph(vpAROgre &ogre, vpPoseVector PoseInit, std::string opath)
{
vpPoseVector PosIn;
vpHomogeneousMatrix cMo;
vpImage<unsigned char> Ior0;
vpImage<unsigned char> Ior1(480,640);
vpImage<unsigned char> Ior2(480,640);
vpImage<vpRGBa> *imArg;
cMo.buildFrom(PoseInit);
vpHomogeneousMatrix oMc;
oMc = cMo.inverse();
vpRotationMatrix Rot;
vpTranslationVector trans;
vpRotationMatrix Rotinv;
vpTranslationVector transinv;
oMc.extract(transinv);
oMc.extract(Rot);
cMo.extract(trans);
cout << "transinv " << trans << endl;
double rho, theta, phi;
//phi = -asin(Rot[0][0]);
/*theta = asin(Rot[0][2]);
//theta = -asin(Rot[2][2]);
phi = asin(Rot[2][0]);
rho = -transinv[0]/sin(phi);*/

rho = sqrt(transinv.sumSquare());
phi = atan(transinv[1]/transinv[0]);
theta = atan(transinv[2]/rho);

double Rho, Theta, Phi, x, y, z;
vpRotationMatrix R1(vpRxyzVector(0,M_PI/2,0));
vpRotationMatrix R2(vpRzyxVector(M_PI/2,0,0));
//vpRotationMatrix R1(vpRxyzVector(0,0,0));
theta=0;
phi=0;
for (int k=0;k<sample_Rho;k++){
for (int l=0;l<sample_Theta;l++){
for (int m=0;m<sample_Phi;m++){
int row=m+(sample_Theta)*l+(sample_Rho*sample_Rho)*k;
Rho=rho;
Theta=theta-(M_PI/2)+l*(2*(M_PI/2)/(sample_Theta-1));
Phi=phi-(M_PI/2)+m*(2*(M_PI/2)/(sample_Phi-1));
Rot[0][0]=-sin(Theta);
Rot[0][1]=cos(Theta)*sin(Phi);
Rot[0][2]=-cos(Phi)*cos(Theta);
Rot[1][0]=cos(Theta);
Rot[1][1]=sin(Phi)*sin(Theta);
Rot[1][2]=-sin(Theta)*cos(Phi);
Rot[2][0]=0;
Rot[2][1]=-cos(Phi);
Rot[2][2]=-sin(Phi);

vpRotationMatrix R2(vpRzyxVector(Theta,0,Phi));
/*transinv[0] = rho*cos(Phi)*sin(Theta);
transinv[1] = rho*sin(Phi)*sin(Theta);
transinv[2] = rho*cos(Theta);*/

//oMc.buildFrom(transinv,R2);

R1 = R2.inverse();

trans[0]=0;
trans[1]=0;
trans[2]=Rho;

/*cMo.buildFrom(trans,R1*Rot);
oMc = cMo.inverse();
oMc.extract(transinv);
oMc.extract(Rot);
oMc.buildFrom(transinv,Rot*R2);*/
//cMo=oMc.inverse();
cMo.buildFrom(trans,R1);


vpImage<unsigned char> *I00 = new vpImage<unsigned char>;;
I00->resize(480,640);
ogre.updateRTTV(*I00,&cMo);
imArg=dt0(I00);
// take square roots
for (int y = 0; y < I00->getHeight(); y++) {
  for (int x = 0; x < I00->getWidth(); x++) {
	  (*imArg)[y][x].A = (*I00)[y][x];
	  Ior1[y][x]=(*imArg)[y][x].B;
	  if((*I00)[y][x]!=100)
	  {
		  Ior2[y][x]=0;
	  }
	  else
	  {
		  Ior2[y][x]=255;
	  }
	  //cout << (double)(*imArg)[y][x].A << endl;
  }
}
/*if(row==153){
vpImageIo::writePPM(Ior1, "DTT.pgm");
vpImageIo::writePPM(Ior2, "EMT.pgm");
}*/
argDTTemp.push_back(imArg);
//delete Ior;
Ior0=*I00;
delete I00;
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), row);
std::string filename(buf);
//std::cout << "Write: " << filename << std::endl;
vpImageIo::writePPM(Ior0, filename);
//vpImageIo::writePPM(*imArg, filename);
		}
		}
		}
		//}
	//}
}

void apViewGeneration::createViewsSphNbhd(vpAROgre &ogre, vpPoseVector PoseInit, std::string opath)
{
vpPoseVector PosIn;
vpHomogeneousMatrix cMo;
vpImage<unsigned char> Ior0;
vpImage<unsigned char> Ior1(480,640);
vpImage<unsigned char> Ior2(480,640);
vpImage<vpRGBa> *imArg;
cMo.buildFrom(PoseInit);
vpHomogeneousMatrix oMc;
oMc = cMo.inverse();
vpRotationMatrix Rot;
vpTranslationVector trans;
vpRotationMatrix Rotinv;
vpTranslationVector transinv;
oMc.extract(transinv);
oMc.extract(Rot);
cMo.extract(trans);
cout << "transinv " << trans << endl;
double rho, theta, phi;
//phi = -asin(Rot[0][0]);
rho = sqrt(trans.sumSquare());
phi = atan(trans[1]/trans[0]);
theta = atan(trans[2]/rho);
//theta = -asin(Rot[2][2]);
//phi = asin(Rot[2][0]);
double Rho, Theta, Phi, x, y, z;
//vpRotationMatrix R1(vpRxyzVector(0,M_PI/2,0));
//vpRotationMatrix R2(vpRzyxVector(M_PI/2,0,0));
//vpRotationMatrix R1(vpRxyzVector(0,0,0));

for (int i = 0; i<nbhd_Theta;i++){

for (int j=0; j<nbhd_Phi;j++){

Theta=theta-(M_PI/2)+i*(2*(M_PI/2)/(nbhd_Theta-1));
Phi=phi-(M_PI/2)+j*(2*(M_PI/2)/(nbhd_Phi-1));

for (int k=0;k<sample_Rho;k++){
for (int l=0;l<snbhd_Theta;l++){
for (int m=0;m<snbhd_Phi;m++){
int row=m+(sample_Theta)*l+(sample_Rho*sample_Rho)*k;
Rho=rho;
Theta = Theta + l*(M_PI/(nbhd_Theta-1))/(snbhd_Theta);
Phi =  Phi + m*(M_PI/(nbhd_Phi-1))/(snbhd_Phi);
vpRotationMatrix R2(vpRzyxVector(M_PI/2+Phi,0,M_PI/2+Theta));
trans[0] = rho*cos(Phi)*sin(Theta);
trans[1] = rho*sin(Phi)*sin(Theta);
trans[2] = rho*cos(Theta);
/*Rot[0][0]=-sin(Theta);
Rot[0][1]=cos(Theta)*sin(Phi);
Rot[0][2]=-cos(Phi)*cos(Theta);
Rot[1][0]=cos(Theta);
Rot[1][1]=sin(Phi)*sin(Theta);
Rot[1][2]=-sin(Theta)*cos(Phi);
Rot[2][0]=0;
Rot[2][1]=-cos(Phi);
Rot[2][2]=-sin(Phi);*/
/*trans[0]=0;
trans[1]=0;
trans[2]=Rho;*/
cMo.buildFrom(trans,R2);
/*cMo.buildFrom(trans,R1*Rot);
oMc = cMo.inverse();
oMc.extract(transinv);
oMc.extract(Rot);
oMc.buildFrom(transinv,Rot*R2);
cMo=oMc.inverse();*/
vpImage<unsigned char> *I00 = new vpImage<unsigned char>;;
I00->resize(480,640);
ogre.updateRTTV(*I00,&cMo);
imArg=dt0(I00);
// take square roots
for (int y = 0; y < I00->getHeight(); y++) {
  for (int x = 0; x < I00->getWidth(); x++) {
	  (*imArg)[y][x].A = (*I00)[y][x];
	  Ior1[y][x]=(*imArg)[y][x].B;
	  if((*I00)[y][x]!=100)
	  {
		  Ior2[y][x]=0;
	  }
	  else
	  {
		  Ior2[y][x]=255;
	  }
	  //cout << (double)(*imArg)[y][x].A << endl;
  }
}
/*if(row==153){
vpImageIo::writePPM(Ior1, "DTT.pgm");
vpImageIo::writePPM(Ior2, "EMT.pgm");
}*/

argDTTemp.push_back(imArg);
//delete Ior;
Ior0=*I00;
delete I00;
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), row);
std::string filename(buf);
//std::cout << "Write: " << filename << std::endl;
vpImageIo::writePPM(Ior0, filename);
//vpImageIo::writePPM(*imArg, filename);
		}
		}
		}
//argDTTempNbhd.push_back(argDTTemp);

/*for (int i = 0; i < argDTTemp.size(); i += 1){
		        if ( argDTTemp[i];!=NULL) delete  argDTTemp[i];
		        argDTTemp[i]=NULL;
		 }
argDTTemp.resize(1);*/
}
}
		//}
	//}
}

void apViewGeneration::invert(vpImage<unsigned char> &IT, vpImage<unsigned char> &Irot)
	{
	int height = IT.getHeight();
	int width = IT.getWidth();
	Irot.resize(height,width);
	double rho,theta;
	int xrot,yrot;
	for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  (Irot)[y][x]=IT[height-1-y][width-1-x];
		  }
	}
}

void apViewGeneration::distanceTransform(vpImage<unsigned char> &I0,vpImage<unsigned char> &I1)
{
vpImage<unsigned char>* I00;
I00=EdgeOrientMaps[1];
//getchar();
for (int i=0;i<I1.getHeight();i++)
{
	for (int j=0;j<I1.getWidth();j++)
	{
		double min=20;
		for (int k=0;k<40;k++)
		{
			for (int l=0;l<40;l++)
			{
					if ( (i-20+k)>0 && (i-20+k)<480 && (j-20+l)>0 && (j-20+l)<640 )
					{
						if (sqrt((k-20)*(k-20)+(l-20)*(l-20))<20 && ((*I00)[i+k-20][j+l-20]<100 || (*I00)[i+k-20][j+l-20]>100))
						{
						if (sqrt((k-20)*(k-20)+(l-20)*(l-20))<min)
						{
							min=(double)sqrt((k-20)*(k-20)+(l-20)*(l-20));
						}
						}
					}
			}
		}
		if (min<50)
		{
		//std::cout<<" ok "<< (double)min <<" ok "<< i << "ok "<< j << std::endl;
		}
		I1[i][j]=(unsigned char)(255*min/20);
	}
	//std::cout<<" ok "<<std::endl;
}
vpImageIo::writePPM(I1, "distT.pgm");
std::cout<<" ok "<<std::endl;
}


void apViewGeneration::dT(vpImage<unsigned char> &I0, vpImage<double> &I1, vpImage<vpRGBa> &I2)
{
	// compute dt
/*vpImage<unsigned char>* I00;
I00=EdgeOrientMaps[n];*/
//I00=&I0;
vpImage<unsigned char>* I00;
I00=&I0;
vpImage<unsigned char> I3(480,640);
vpImage<double> *out;
vpImage<vpRGBa> *imArg = new vpImage<vpRGBa>;
imArg->resize(480,640);
I1.resize(480,640);
I2.resize(480,640);
//std::cout<<" ok "<<std::endl;
out=dt(I00,imArg);
// take square roots
for (int y = 0; y < (&I0)->getHeight(); y++) {
  for (int x = 0; x < (&I0)->getWidth(); x++) {
	  //(*out)[y][x] = sqrt((*out)[y][x]);
	  //cout << (double)(*out)[y][x] << endl;
	  I1[y][x]=sqrt((*out)[y][x]);
	  //I3[y][x]=(unsigned char)(*out)[y][x];
	  (I2[y][x]).R=((*imArg)[y][x]).R;
	  (I2[y][x]).G=((*imArg)[y][x]).G;
	  //(I2[y][x]).B=(unsigned char) sqrt((*out)[y][x]);
	  //(I2[y][x]).A=(*I00)[y][x];
	  //std::cout<<" ok "<< (double)I1[y][x] << std::endl;
  }
}
//vpImageIo::writePPM(I3, "ImdT.pgm");
//std::cout<<" ok0 "<<std::endl;
}

/*vpImage<vpRGBa> apViewGeneration::dT0(vpImage<unsigned char> &I0)
{

vpImage<unsigned char>* I00;
I00=&I0;
//vpImage<unsigned char> I3(480,640);
//vpImage<double> *out;
//I2.resize(480,640);
//std::cout<<" ok "<<std::endl;
imArg=dt0(I00);
// take square roots
for (int y = 0; y < (&I0)->getHeight(); y++) {
  for (int x = 0; x < (&I0)->getWidth(); x++) {

	  (I2[y][x]).R=((*imArg)[y][x]).R;
	  (I2[y][x]).G=((*imArg)[y][x]).G;
	  (I2[y][x]).B=((*imArg)[y][x]).B;
	  (I2[y][x]).A=(*I00)[y][x];
	  //std::cout<<" ok "<< (double)I1[y][x] << std::endl;
  }
}
//vpImageIo::writePPM(I3, "ImdT.pgm");
//std::cout<<" ok0 "<<std::endl;
}*/

void apViewGeneration::dTTemplates()
{
vpImage<unsigned char>* I00;
vpImage<vpRGBa> *imArg;
vpImage<double> *out;
	// compute dt
	for (int n = 0; n<EdgeOrientMaps.size()-1;n++)
	{
I00=EdgeOrientMaps[n+1];
//I00=&I0;
imArg = new vpImage<vpRGBa>;
imArg->resize(480,640);
out=dt(I00,imArg);
// take square roots
for (int y = 0; y < I00->getHeight(); y++) {
  for (int x = 0; x < I00->getWidth(); x++) {
	  (*out)[y][x] = sqrt((*out)[y][x]);
	  /*if (sqrt((*out)[y][x])>255) (*imArg)[y][x].B = 255;
	  else
	  {
	  (*imArg)[y][x].B = (unsigned char) sqrt((*out)[y][x]);
	  }
	  (*imArg)[y][x].A = (*I00)[y][x];*/

  }
}
//cout << (double)(*out)[22][304] << (double)(((*imArg)[22][304]).R-127)*2 << " " <<(double)(((*imArg)[22][304]).G-127)*2<< endl;
DTTemp.push_back(out);
argDTTemp.push_back(imArg);
//delete imArg;
//delete out;
/*std::string opath = "I00%04d.pgm";
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), n);
std::string filename(buf);
vpImageIo::writePPM((*I00), filename);*/
//cout << " ok "<< n<< endl;
	}
//vpImageIo::writePPM(I1, "distT.pgm");
}

void apViewGeneration::dTTemplates0()
{
vpImage<unsigned char>* I00;
vpImage<vpRGBa> *imArg;
//vpImage<double> *out;
	// compute dt
	for (int n = 0; n<EdgeOrientMaps.size()-1;n++)
	{
I00=EdgeOrientMaps[n+1];
//imArg = new vpImage<vpRGBa>;
imArg=dt0(I00);
// take square roots
for (int y = 0; y < I00->getHeight(); y++) {
  for (int x = 0; x < I00->getWidth(); x++) {
	  //(*out)[y][x] = sqrt((*out)[y][x]);
	  (*imArg)[y][x].A = (*I00)[y][x];

  }
}

argDTTemp.push_back(imArg);

	}
}


/* dt of 1d function using squared distance */
float* apViewGeneration::dtx(float *f, float *g, int n, int *argx, int *argy) {
  float *d = new float[n];
  int *v = new int[n];
  float *z = new float[n+1];
  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = +INF;
  for (int q = 1; q <= n-1; q++) {
    float s  = ((f[q]+vpMath::sqr(q))-(f[v[k]]+vpMath::sqr(v[k])))/(2*q-2*v[k]);
    while (s <= z[k]) {
      k--;
      s  = ((f[q]+vpMath::sqr(q))-(f[v[k]]+vpMath::sqr(v[k])))/(2*q-2*v[k]);
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k+1] = +INF;
  }

  k = 0;
  for (int q = 0; q <= n-1; q++) {
    while (z[k+1] < q)
      k++;
    d[q] = vpMath::sqr(q-v[k]) + f[v[k]];
    argx[q] = q-v[k];
    argy[q] = g[v[k]];
  }

  delete [] v;
  delete [] z;
  return d;
}

float* apViewGeneration::dty(float *f, float *g, int n) {
  float *d = new float[n];
  int *v = new int[n];
  float *z = new float[n+1];
  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = +INF;
  for (int q = 1; q <= n-1; q++) {
    float s  = ((f[q]+vpMath::sqr(q))-(f[v[k]]+vpMath::sqr(v[k])))/(2*q-2*v[k]);
    while (s <= z[k]) {
      k--;
      s  = ((f[q]+vpMath::sqr(q))-(f[v[k]]+vpMath::sqr(v[k])))/(2*q-2*v[k]);
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k+1] = +INF;
  }

  k = 0;
  for (int q = 0; q <= n-1; q++) {
    while (z[k+1] < q)
      k++;
    d[q] = vpMath::sqr(q-v[k]) + f[v[k]];
    g[q] = q-v[k];
  }

  delete [] v;
  delete [] z;
  return d;
}

/* dt of 2d function using squared distance */
void apViewGeneration::dtf(vpImage<double> *im, vpImage<vpRGBa> *imArg) {
  int width = im->getWidth();
  int height = im->getHeight();
  float *f = new float[std::max(width,height)];
  float *g = new float[std::max(width,height)];
  vpColVector arg(2);
  // transform along columns
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      f[y] = (float)(*im)[y][x];
    }
    //int *argy = new int[height];
    float *d = dty(f,g, height);
    for (int y = 0; y < height; y++) {
    	 (*im)[y][x] = (double)d[y];
         ((*imArg)[y][x]).R = g[y]/2+127;
        /* if (argy[y] <10000)
         cout << sqrt(argy[y]) << endl;*/
    }
    delete [] d;
    //delete [] argy;
  }
  // transform along rows
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      f[x] =  (float)(*im)[y][x];
      g[x] =  ((*imArg)[y][x]).R;
    }
    int *argx = new int[width];
    int *argy = new int[width];
    float *d = dtx(f,g, width, argx, argy);
    for (int x = 0; x < width; x++) {
    	 (*im)[y][x]= (double)d[x];
    	 ((*imArg)[y][x]).R = argy[x];
    	 ((*imArg)[y][x]).G = argx[x]/2+127;
    }
    delete [] d;
    delete [] argx;
    delete [] argy;
  }
  delete f;
  delete g;
}


void apViewGeneration::dtf0(vpImage<double> *im, vpImage<vpRGBa> *imArg) {
  int width = im->getWidth();
  int height = im->getHeight();
  float *f = new float[std::max(width,height)];
  float *g = new float[std::max(width,height)];
  vpColVector arg(2);
  // transform along columns
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      f[y] = (float)(*im)[y][x];
    }
    //int *argy = new int[height];
    float *d = dty(f,g, height);
    for (int y = 0; y < height; y++) {
    	 (*im)[y][x] = (double)d[y];
         ((*imArg)[y][x]).R = g[y]/2+127;
        /* if (argy[y] <10000)
         cout << sqrt(argy[y]) << endl;*/
    }
    delete [] d;
    //delete [] argy;
  }
  // transform along rows
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      f[x] =  (float)(*im)[y][x];
      g[x] =  ((*imArg)[y][x]).R;
    }
    int *argx = new int[width];
    int *argy = new int[width];
    float *d = dtx(f,g, width, argx, argy);
    for (int x = 0; x < width; x++) {
    	if(sqrt(d[x])<255)
    	 (*imArg)[y][x].B= sqrt(d[x]);
    	else (*imArg)[y][x].B = 255;
    	 ((*imArg)[y][x]).R = argy[x];
    	 ((*imArg)[y][x]).G = argx[x]/2+127;
    }
    delete [] d;
    delete [] argx;
    delete [] argy;
  }
  delete f;
  delete g;
}


/* dt of binary image using squared distance */
vpImage<double>* apViewGeneration::dt(vpImage<unsigned char> *im, vpImage<vpRGBa> *imArg){
  int width = im->getWidth();
  int height = im->getHeight();

  vpImage<double> *out = new vpImage<double>;
  out->resize(480,640);
  //int argdt[height][width][2];
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if ((*im)[y][x]==100)
      {
	(*out)[y][x] = INF;
      }
      else
      {
    (*out)[y][x] = 0;
      }
    }
  }
  //vpImageIo::writePPM(*out, "distT1.pgm");
  dtf(out, imArg);
  return out;
}

vpImage<vpRGBa>* apViewGeneration::dt0(vpImage<unsigned char> *im){
  int width = im->getWidth();
  int height = im->getHeight();

  vpImage<double> *out = new vpImage<double>;
  vpImage<vpRGBa> *imArg = new vpImage<vpRGBa>;
  out->resize(480,640);
  imArg->resize(480,640);
  //int argdt[height][width][2];
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if ((*im)[y][x]==100)
      {
	(*out)[y][x] = INF;
      }
      else
      {
    (*out)[y][x] = 0;
      }
    }
  }
  //vpImageIo::writePPM(*out, "distT1.pgm");
  dtf0(out, imArg);
  return imArg;
}


void apViewGeneration::edgeOrientMap(vpImage<unsigned char> &I0)
{
int width = I0.getWidth();
int height = I0.getHeight();
vpImage<unsigned char> I1(height,width);
/*double cannyTh1=60;
double cannyTh2=120;*/
double cannyTh1=60;
double cannyTh2=120;
IplImage* Ip = NULL;
vpImageConvert::convert(I0, Ip);
IplImage* dst = cvCreateImage( cvSize(I0.getWidth(),I0.getHeight()), 8, 1 );
cvCanny( Ip, dst, cannyTh1, cannyTh2, 3 );
vpImageConvert::convert(dst, I1);
/*vector<Vec4i> lines;
Mat dst0(dst);
Mat color_dst;
double t0= vpTime::measureTimeMs();
HoughLinesP( dst0, lines, 1, CV_PI/120, 50, 30, 10 );
double t1= vpTime::measureTimeMs();
cout << " timem " << t1-t0 << endl;
vpImageConvert::convert(dst, I1);
vpImageIo::writePPM(I0, "Inorm.pgm");
vpImageIo::writePPM(I1, "Canny.pgm");

cvtColor( dst0, color_dst, CV_GRAY2BGR );
for( size_t i = 0; i < lines.size(); i++)
    {
        line( color_dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 2, 8 );
    }
cout<<lines[0][0]<<" "<<lines[0][1]<<" "<<lines[0][2]<<" "<<lines[0][3]<<endl;
//LEx.setLines(lines,zbuf,cam);
//Lines=LEx.getLines();

//cout<<lines.size()<<endl;
    IplImage* Ip1=new IplImage(color_dst);
    vpImageConvert::convert(Ip1,I0);
    vpImageIo::writePPM(I0, "Hough.pgm");*/



vpImage<double> Iu;
vpImage<double> Iv;
vpImage<double> imG(height,width);
double a,b;
int n,m;

for (n=3; n < height-3 ; n++)
{
for (m = 3 ; m < width-3; m++)
  {
    imG[n][m] =   apImageFilter::gaussianFilter(I0,n,m);
  }
}

for (int i=3;i<height-3;i++)
		{
			for (int j=3;j<width-3;j++)
			{
				a=(apImageFilter::sobelFilterX(imG,i,j));
				b=(apImageFilter::sobelFilterY(imG,i,j));
			    if ( I1[i][j]==255) //
				{

				 I0[i][j]=255*(-(atan(a/b))/M_PI+1/2);

				}
				 else {I0[i][j]=100;}
			}
	}

//vpImageIo::writePPM(I0, "edgeor.pgm");

}

void apViewGeneration::computeSimilarity(vpImage<unsigned char> &I0)
{
edgeOrientMap(I0);
vpImage<vpRGBa> argImDT(480,640);
vpImage<double> ImDT(480,640);
cout  <<" ok " << endl;
dT(I0,ImDT,argImDT);
//vpImage<unsigned char> I00(480,640);
//vpImage<unsigned char> *I10;

vpImage<unsigned char> *I00;
vpImage<vpRGBa> *argIDT;
vpImage<double> *IDT;

//I10=EdgeOrientMaps[1];
//vpImageIo::writePPM((*I10), "edge.pgm");
//I10=&I0;
//I00=&I0;
int width = I0.getWidth();
int height = I0.getHeight();
double distT[EdgeOrientMaps.size()-1];
double cor;
double t0,t1;
t0= vpTime::measureTimeMs();
for (int n=0 ; n<EdgeOrientMaps.size()-1;n++)
{



//vpImage<vpRGBa> *argIDT = new vpImage<vpRGBa>;

//vpImage<double> *IDT = new vpImage<double>;
//vpImage<unsigned char> *I00;
argIDT=argDTTemp[n+1];
IDT=DTTemp[n+1];
I00=EdgeOrientMaps[n+1];
//vpImageIo::writePPM((*I11), "edge.pgm");
/*std::string opath = "I00%04d.pgm";
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), n);
std::string filename(buf);
vpImageIo::writePPM(I0, filename);*/
double dist=0;
int m=0;
int mo=0;
double delta;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {
    if (((*I00)[y][x]<100 || (*I00)[y][x]>100))
	  //if (((*argIDT)[y][x].A<100 || (*argIDT)[y][x].A>100))
    {
    	m++;
        //double coo = (double)(((ImDT)[y][x]));
    	/*double coo1 = (double)(((*argIDT)[20][20]).R-127)*2;
    	cout  <<" ok " << coo1 << endl;*/
    	double orientT = (double)(I0)[y-(((argImDT)[y][x]).R-127)*2][x-(((argImDT)[y][x]).G-127)*2];
    	//getchar();
    	//double orientT = (double)(argImDT)[y-(((argImDT)[y][x]).R-127)*2][x-(((argImDT)[y][x]).G-127)*2].A;
    	int yy=-1;
    	int xx=-1;
    	/*while (orientT==100 && yy<2)
    		{
    		orientT=(double)(I0)[y-(((argImDT)[y][x]).R-127)*2+yy][x-(((argImDT)[y][x]).G-127+xx)*2];
    		yy ++;
			while ( orientT==100 && xx<2)
			{
				orientT=(double)(I0)[y-(((argImDT)[y][x]).R-127)*2+yy][x-(((argImDT)[y][x]).G-127+xx)*2];
				xx ++;
			}
    		}*/
    	if (orientT>100 || orientT<100)
    		{
    		mo++;
    		delta=abs((*I00)[y][x]-orientT);
    		//delta=abs((double)(*argIDT)[y][x].A-orientT);
    		if (delta>255-delta) delta=255-delta;
	   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
    		//cout  <<" " << delta << endl;
    	dist = dist + (ImDT)[y][x] + 0.1*delta;
    		//dist = dist + (double)(argImDT)[y][x].B + 0.1*delta;
    	//dist = dist  + 0.1*delta;
    		}
    	/*else
    	dist = dist + (ImDT)[y][x];*/
    }
  }
}
cout << " m " << m << " " << mo << " n " << n << endl;
//delete argIDT;
//delete IDT;
cout << dist/mo << endl;
distT[n]=dist;
//if (n==7) getchar();
}

t1= vpTime::measureTimeMs();
cout << " timem " << t1-t0 << endl;

}


void apViewGeneration::computeSimilarity0(vpImage<unsigned char> &I0, std::string opath)
{
edgeOrientMap(I0);
vpImage<vpRGBa> *argImDT;
//vpImage<double> ImDT(480,640);
cout  <<" ok " << endl;
//dT0(I0,*argImDT);
vpImage<unsigned char>* Im;
Im=&I0;
//vpImage<unsigned char> I3(480,640);
//vpImage<double> *out;
//I2.resize(480,640);
//std::cout<<" ok "<<std::endl;
argImDT=dt0(Im);
//vpImage<unsigned char> *I00;
vpImage<unsigned char> I10(480,640);
vpImage<vpRGBa> *argIDT;
//vpImage<double> *IDT;
vpImage<unsigned char> *IT;
//I10=EdgeOrientMaps[1];
//vpImageIo::writePPM((*I10), "edge.pgm");
//I10=&I0;
//I00=&I0;
int width = I0.getWidth();
int height = I0.getHeight();
int m,mo,yy,xx,adtIy,adtIx;
double distT[EdgeOrientMaps.size()-1];
double cor,t0,t1,dist,delta,oriT,oriI,dtI;
vpRGBa IDT, ImDT;

t0= vpTime::measureTimeMs();
//for (int n=0 ; n<EdgeOrientMaps.size()-1;n++)
for (int n=0 ; n<80;n++)
{
IT = new vpImage<unsigned char>;
IT->resize(480,640);
//argIDT=argDTTemp[n+1];
	cout  <<" ok0 " << endl;
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), n);
std::string filename(buf);
//std::cout << "Write: " << filename << std::endl;
vpImageIo::readPPM(*IT, filename);
cout  <<" ok1 " << filename << endl;
//IT=EdgeOrientMaps[n+1];
//vpImageIo::writePPM((*I11), "edge.pgm");
/*std::string opath = "I00%04d.pgm";
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), n);
std::string filename(buf);
vpImageIo::writePPM(I0, filename);*/
dist=0;
int m=0;
int mo=0;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {

	  //oriT=(double)(*argIDT)[y][x].A;
	  /*oriT=(double)(*IT)[y][x];
	  adtIy = y-(((*argImDT)[y][x]).R-127)*2;
	  adtIx = x-(((*argImDT)[y][x]).G-127)*2;
	  dtI = (double)(*argImDT)[y][x].B;*/
	  //cout << "ok " << x << " "<< y << " "<< (double)adtIy << endl;

    //if (oriT<100 || oriT>100)
	  if ((*IT)[y][x]<100 || (*IT)[y][x]>100)
    {
    	m++;
    	//oriI = (double)(*Im)[adtIy][adtIx];
    	oriI = (double)(*Im)[y-(((*argImDT)[y][x]).R-127)*2][x-(((*argImDT)[y][x]).G-127)*2];
    	//getchar();
    	//double oriT = (double)(argImDT)[y-(((argImDT)[y][x]).R-127)*2][x-(((argImDT)[y][x]).G-127)*2].A;
    	yy=-1;
    	xx=-1;
    	/*while (oriT==100 && yy<2)
    		{
    		oriT=(double)(I0)[y-(((argImDT)[y][x]).R-127)*2+yy][x-(((argImDT)[y][x]).G-127+xx)*2];
    		yy ++;
			while ( oriT==100 && xx<2)
			{
				oriT=(double)(I0)[y-(((argImDT)[y][x]).R-127)*2+yy][x-(((argImDT)[y][x]).G-127+xx)*2];
				xx ++;
			}
    		}*/
    	if (oriI>100 || oriI<100)
    		{
    		mo++;
    		//delta=abs(oriT-oriI);
    		delta=abs((*IT)[y][x]-oriI);
    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
    		if (delta>255-delta) delta=255-delta;
	   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
    		//cout  <<" " << delta << endl;
    	//dist = dist + dtI + 0.1*delta;
    		dist = dist + (double)(*argImDT)[y][x].B + 0.2*delta;
    	//dist = dist  + 0.1*delta;
    		}
    	/*else
    	dist = dist + (ImDT)[y][x];*/
    }
  }
}
cout << " m " << m << " " << mo << " n " << n << endl;
//delete argIDT;
//delete IDT;
cout << dist/mo << endl;
//distT[n]=dist;
//if (n==7) getchar();
delete IT;
}

t1= vpTime::measureTimeMs();
cout << " timem " << t1-t0 << endl;

}

void apViewGeneration::merge()
{
//vpImage<unsigned char> *I001;
vpImage<vpRGBa> *I0;
//vpImage<double> *IDT1;
I0=argDTTemp[1];
//IDT1=DTTemp[1];
//I001=EdgeOrientMaps[1];
//vpImage<unsigned char> *I002;
vpImage<vpRGBa> *I1;
//vpImage<double> *IDT2;
I1=argDTTemp[69];
/*IDT2=DTTemp[69];
I002=EdgeOrientMaps[69];*/
vpImage<unsigned char> I003(480,640);
vpImage<vpRGBa> I3(480,640);
//vpImage<unsigned char> IDT3(480,640);
for (int y = 0; y < 480; y++) {
  for (int x = 0; x < 640; x++) {
	I003[y][x]=100;
	I3[y][x].A=100;
  }
  }
int m=0;
for (int y = 0; y < 480-0; y++) {
  for (int x = 0; x < 640-0; x++) {
I3[y][x].R=0.5*((*I0)[y][x].R+(*I1)[y][x].R);
I3[y][x].G=0.5*((*I0)[y][x].G+(*I1)[y][x].G);
if (0.5*((*I0)[y][x].B+(*I1)[y][x].B)>255) I3[y][x].B = 255;
else
I3[y][x].B = 0.5*((*I0)[y][x].B+(*I1)[y][x].B);
int I0y = ((*I0)[y][x].R-127)*2;
int I0x = ((*I0)[y][x].G-127)*2;
int I1y = ((*I1)[y][x].R-127)*2;
int I1x = ((*I1)[y][x].G-127)*2;
//cout << x << " " << y << " " << (double)I0y << " " << (double)(*I0)[y][x].B  << endl;
if((*I0)[y][x].B<255 && (*I1)[y][x].B<255){
if (((*I0)[y-I0y][x-I0x].A>100 || (*I0)[y-I0y][x-I0x].A<100) && ((*I1)[y-I1y][x-I1x].A>100 || (*I1)[y-I1y][x-I1x].A<100))
{I3[y-(I3[y][x].R-127)*2][x-(I3[y][x].G-127)*2].A = 0.5*((*I0)[y-I0y][x-I0x].A+(*I1)[y-I1y][x-I1x].A);
I003[y-(I3[y][x].R-127)*2][x-(I3[y][x].G-127)*2] = 0.5*((*I0)[y-I0y][x-I0x].A+(*I1)[y-I1y][x-I1x].A);
m++;
}
}

}
}
cout << m << endl;
vpImageIo::writePPM(I003,"merge.pgm");

}

vpImage<vpRGBa>* apViewGeneration::merge(vpImage<vpRGBa> *I0, vpImage<vpRGBa> *I1)
{
int width = I1->getWidth();
int height = I1->getHeight();
vpImage<unsigned char> I003(height,width);
vpImage<vpRGBa> *I3 = new vpImage<vpRGBa>;
std::cout << " height " << height << " width " << width << std::endl;
I3->resize(height,width);
//vpImage<unsigned char> IDT3(480,640);
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {
	//I003[y][x]=100;
	(*I3)[y][x].A=100;
  }
  }
int m=0;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {
	  if((*I0)[y][x].B<255 && (*I1)[y][x].B<255){
(*I3)[y][x].R=0.5*((*I0)[y][x].R+(*I1)[y][x].R);
(*I3)[y][x].G=0.5*((*I0)[y][x].G+(*I1)[y][x].G);
/*if (0.5*((*I0)[y][x].B+(*I1)[y][x].B)>255) (*I3)[y][x].B = 255;
else*/
(*I3)[y][x].B = 0.5*((*I0)[y][x].B+(*I1)[y][x].B);
int I0y = ((*I0)[y][x].R-127)*2;
int I0x = ((*I0)[y][x].G-127)*2;
int I1y = ((*I1)[y][x].R-127)*2;
int I1x = ((*I1)[y][x].G-127)*2;
//cout << x << " " << y << " " << (double)((*argIDT1)[y][x].R-127)*2 << " " << (double)(*argIDT2)[y][x].R  << endl;

//cout << x << " " << y << " " << I0y << " " << I0x << " "  << I1y << " " << I1x  << endl;
if(y-I0y>0 && y-I0y<480 && y-I1y>0 && y-I1y<480 && x-I0x>0 && x-I0x<640 && x-I1x<640 && x-I1x>0)
{
if (((*I0)[y-I0y][x-I0x].A!=100) && ((*I1)[y-I1y][x-I1x].A!=100 ))
{(*I3)[y-((*I3)[y][x].R-127)*2][x-((*I3)[y][x].G-127)*2].A = 0.5*((*I0)[y-I0y][x-I0x].A+(*I1)[y-I1y][x-I1x].A);
I003[y-((*I3)[y][x].R-127)*2][x-((*I3)[y][x].G-127)*2] = 0.5*((*I0)[y-I0y][x-I0x].A+(*I1)[y-I1y][x-I1x].A);
m++;
}
}
}
	  else
	  {
		  (*I3)[y][x].B=255;
	  }


}
}
std::cout << " height0 " << height << " width0 " << width << std::endl;
//vpImageIo::writePPM(I003,"merge.pgm");
return I3;
}

vpImage<vpRGBa>* apViewGeneration::mergeScale(vpImage<vpRGBa> *I0, vpImage<vpRGBa> *I1, int scale)
{
int width = I0->getWidth();
int height = I0->getHeight();
vpImage<unsigned char> I003(height,width);
vpImage<vpRGBa> *I3 = new vpImage<vpRGBa>;
std::cout << " height " << height << " width " << width << std::endl;
I3->resize(height,width);
unsigned int cScale = static_cast<unsigned int>(pow(2., (int)(scale)));
int offs = (int)(127/cScale);
//vpImage<unsigned char> IDT3(480,640);
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {
	I003[y][x]=100;
	(*I3)[y][x].A=100;
  }
  }
int m=0;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {
	  if((*I0)[y][x].B<2*offs && (*I1)[y][x].B<2*offs){
(*I3)[y][x].R=0.5*((*I0)[y][x].R+(*I1)[y][x].R);
(*I3)[y][x].G=0.5*((*I0)[y][x].G+(*I1)[y][x].G);
/*if (0.5*((*I0)[y][x].B+(*I1)[y][x].B)>255) (*I3)[y][x].B = 255;
else*/
(*I3)[y][x].B = 0.5*((*I0)[y][x].B+(*I1)[y][x].B);
int I0y = ((*I0)[y][x].R-offs)*2;
int I0x = ((*I0)[y][x].G-offs)*2;
int I1y = ((*I1)[y][x].R-offs)*2;
int I1x = ((*I1)[y][x].G-offs)*2;
//std::cout << " I1x " << I1x << " I1y " << I1y << std::endl;
//cout << x << " " << y << " " << (double)((*argIDT1)[y][x].R-127)*2 << " " << (double)(*argIDT2)[y][x].R  << endl;

	//cout << x << " " << y << " " << I0y << " " << I0x  << endl;
if (((*I0)[y-I0y][x-I0x].A>100 || (*I0)[y-I0y][x-I0x].A<100) && ((*I1)[y-I1y][x-I1x].A>100 || (*I1)[y-I1y][x-I1x].A<100))
{(*I3)[y-((*I3)[y][x].R-offs)*2][x-((*I3)[y][x].G-offs)*2].A = 0.5*((*I0)[y-I0y][x-I0x].A+(*I1)[y-I1y][x-I1x].A);
I003[y-((*I3)[y][x].R-offs)*2][x-((*I3)[y][x].G-offs)*2] = 0.5*((*I0)[y-I0y][x-I0x].A+(*I1)[y-I1y][x-I1x].A);
m++;
}
}
	  else
	  {
		  (*I3)[y][x].B=2*offs;
	  }

}
}
std::cout << " height0 " << height << " width0 " << width << std::endl;
//vpImageIo::writePPM(I003,"merge.pgm");
return I3;
}


void apViewGeneration::buildHierarchy(double threshold, const char *filename)
{
vpImage<vpRGBa> *I0;
vpImage<vpRGBa> *I1;
vpImage<vpRGBa> *I2;
vpImage<vpRGBa> *I3;
vpImage<vpRGBa> *I4;
vpImage<vpRGBa> *I5;
vpImage<vpRGBa> *I6;
vpImage<vpRGBa> *mv;
vpImage<vpRGBa> *mv0;
vpImage<unsigned char> I(480,640);
int nm,lm;
lm=1;
int indice;
double sim;
double simax;
vpMatrix hierarchy(10000,4);

/*std::fstream file;
//if (!binary)
     file.open(filename, std::fstream::out);*/
  /* else
     file.open(filename, std::fstream::out|std::fstream::binary);*/

/*   if(!file)
   {
     file.close();
   }

   else
   {
     if (!binary)
     {
       int i = 0;
       file << "# ";
       while (Header[i] != '\0')
       {
     file << Header[i];
     if (Header[i] == '\n')
       file << "# ";
     i++;
       }
       file << '\0';
       file << std::endl;
       file << M.getRows() << "\t" << M.getCols() << std::endl;
       file << M << std::endl;
     }*/


vpColVector inc;
vpColVector inc0;
int level=0;
int nrow=0;
int npl,ntemp;
vpColVector hiertemp;
std::string opath = "/local/agpetit/images" + vpIoTools::path("/imagesSoyuz/Hierarchy/I%04d.pgm");
char buf0[FILENAME_MAX];
while (lm>0 && argDTTemp.size()>4)
{
	hiertemp.resize(argDTTemp.size());
	ntemp=0;
	//bool inc[argDTTemp.size()];
	inc.resize(argDTTemp.size(),true);
	inc0.resize(argDTTemp.size(),true);
cout << " lm " <<  lm << endl;
	lm=0;
npl=1;
//int mm=0;
		for (int n = 0;n<argDTTemp.size()-2;n++)
	{

			/*if (inc0[n+1]==0)
			{*/

			//cout << " ooooooookkk5 "  <<  endl;
		I0=argDTTemp[n+1];
        //mm++;
		nm=0;
		indice=0;
		simax = threshold;
		for (int k=n+1 ; k<argDTTemp.size()-1;k++)
		{

			I1=argDTTemp[k+1];
			sim = computeSimilarity(I0,I1);

			//cout << " ooooooookkk3 " << sim <<  endl;
			if (sim<threshold && sim<simax)
			{
				simax=sim;
				indice=k+1;
				//cout << " ooooooookkk3 " << " " << n+1<< " " << k+1 <<  endl;
			}
		}

        if(indice>0)
        {
        I1=argDTTemp[indice];
		mv=merge(I0,I1);
		inc[indice]=1;
		//inc0[indice]=1;
		//cout << " ooooooookkk4 " <<  endl;
		mergeTemp.push_back(mv);
		/*file << level << " "<< n+1 << " " << indice;
		file << std::endl;*/
		hierarchy[nrow][0] = level;
		hierarchy[nrow][1] = n+1;
		hierarchy[nrow][2] = indice;
		hierarchy[nrow][3] = npl;
		nrow++;
		npl++;

		for (int y=0; y<I0->getHeight();y++)
		{
			for (int x=0;x<I0->getWidth();x++)
			{
		I[y][x]=(*mv)[y][x].A;
			}
		}

		//sprintf(buf0, opath.c_str(), (level+1)*1000 + n+1);
		sprintf(buf0, opath.c_str(), (level+1)*1000 + lm+1);
		std::string filename0(buf0);
		vpImageIo::writePPM(I, filename0);

		nm++;
		lm++;
        }


		//cout << " ooooooookkk3 " << nm <<  endl;
		if (nm==0 && inc[n+1]==0)
		{
			/*I2=new vpImage<vpRGBa>;
			I2->resize(480,640);
			*I2=*I0;*/
			//mergeTemp.push_back(I2);
			/*file << level << " "<< n+1 << " " << 0;
			file << std::endl;*/
			hiertemp[ntemp]=n+1;
			ntemp++;
			/*hierarchy[nrow][0] = level;
			hierarchy[nrow][1] = n+1;
			hierarchy[nrow][2] = 0;
			hierarchy[nrow][3] = npl;
			nrow++;
			npl++;*/

			/*for (int y=0; y<I0->getHeight();y++)
			{
				for (int x=0;x<I0->getWidth();x++)
				{
			I[y][x]=(*I0)[y][x].A;
				}
			}

			sprintf(buf0, opath.c_str(), (level+1)*1000 + n+1);
			std::string filename1(buf0);
			vpImageIo::writePPM(I, filename1);*/
		}

	//}

	}

		for (int k=0;k<ntemp;k++)
		{
			int nl = hiertemp[k];
			I1=argDTTemp[nl];

			hierarchy[nrow][0] = level;
			hierarchy[nrow][1] = nl;
			hierarchy[nrow][2] = 0;
			hierarchy[nrow][3] = npl;
			nrow++;
			npl++;


			for (int y=0; y<I1->getHeight();y++)
						{
							for (int x=0;x<I1->getWidth();x++)
							{
						I[y][x]=(*I1)[y][x].A;
							}
						}

						sprintf(buf0, opath.c_str(), (level+1)*1000 + lm+1+k);
						std::string filename1(buf0);
						vpImageIo::writePPM(I, filename1);



		}




		/*if (inc[argDTTemp.size()-1]==0)
		{
			I1=argDTTemp[argDTTemp.size()-1];
			I6=new vpImage<vpRGBa>;
			I6->resize(480,640);
			*I6=*I1;
			mergeTemp.push_back(I6);
			hierarchy[nrow][0] = level;
			hierarchy[nrow][1] = argDTTemp.size()-1;
			hierarchy[nrow][2] = 0;
			hierarchy[nrow][3] = npl;
			nrow++;
			npl++;

			for (int y=0; y<I0->getHeight();y++)
			{
				for (int x=0;x<I0->getWidth();x++)
				{
			I[y][x]=(*I1)[y][x].A;
				}
			}

			sprintf(buf0, opath.c_str(), (level+1)*1000 + argDTTemp.size()-1);
			std::string filename1(buf0);
			vpImageIo::writePPM(I, filename1);


		}*/

	for (int i = 0; i < argDTTemp.size(); i += 1){
			        I4 = argDTTemp[i];
			        if (I4!=NULL) delete I4;
			        I4=NULL;
			 }
	    argDTTemp.resize(1);
	    //argDTTemp=mergeTemp;
	   cout << " ooooooookkk1 " << mergeTemp.size() << endl;
	  for (int i = 0; i < mergeTemp.size()-1; i += 1){
	  //for (int i = 0; i < floor(mergeTemp.size()/10); i += 1){
		  I3=new vpImage<vpRGBa>;
		  I3->resize(480,640);
		  *I3=*mergeTemp[i+1];
		  argDTTemp.push_back(I3);
	  }

	 for (int i = 0; i < mergeTemp.size(); i += 1){
		  //I3=new vpImage<vpRGBa>;
		  //I3->resize(480,640);
	        mv0 = mergeTemp[i];
		  //*I3=*mv0;
		 //argDTTemp.push_back(I2);
		  //argDTTemp.push_back(mv0);
	        if (mv0!=NULL) delete mv0;
	        mv0=NULL;
		  //delete I2;
	 }
	  mergeTemp.resize(1);
	  I5=argDTTemp[3];
	  cout << " ooooooookkk2 " <<  argDTTemp.size() << endl;
	  /*for (int y=0; y<I5->getHeight();y++)
	  {
	  	for (int x=0;x<I5->getWidth();x++)
	  	{
	  		//cout << " ooooooookkk " << (double)(*I3)[y][x].A<<endl;
	  I[y][x]=(*I5)[y][x].A;
	  	}
	  }*/
	  //vpImageIo::writePPM(I, "merge1.pgm");

	  /*for (int n = 0;n<argDTTemp.size()-1;n++)
	  {
	  I0=argDTTemp[n+1];
	  for (int y=0; y<I0->getHeight();y++)
	  {
	  	for (int x=0;x<I0->getWidth();x++)
	  	{
	  I[y][x]=(*I0)[y][x].A;
	  	}
	  }

	  char buf[FILENAME_MAX];
	  sprintf(buf, opath.c_str(), n);
	  std::string filename(buf);
	  vpImageIo::writePPM(I, filename);

	  }*/

level++;
}
cout << " okkok" << endl;
hierarchy.resize(nrow,4,false);
hierarchy.saveMatrix(filename,hierarchy,false);
//file.close();
/*for (int n = 0;n<argDTTemp.size()-1;n++)
{
I0=argDTTemp[n+1];
for (int y=0; y<I0->getHeight();y++)
{
	for (int x=0;x<I0->getWidth();x++)
	{
I[y][x]=(*I0)[y][x].A;
	}
}

char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), n);
std::string filename0(buf);
vpImageIo::writePPM(I, filename0);

}*/


}


void apViewGeneration::buildHierarchy1(double threshold, const char *filename)
{
vpImage<vpRGBa> *I0;
vpImage<vpRGBa> *I1;
vpImage<vpRGBa> *I2;
vpImage<vpRGBa> *I3;
vpImage<vpRGBa> *I4;
vpImage<vpRGBa> *I5;
vpImage<vpRGBa> *I6;
vpImage<vpRGBa> *mv;
vpImage<vpRGBa> *mv0;
vpImage<unsigned char> I(480,640);
int nm,lm;
lm=1;
int flag = 0;
int indice;
double sim;
double simax;
vpMatrix hierarchy(10000,40);

vpColVector inc;
vpColVector inc0;
int level=0;
int nrow=0;
int npl,ntemp;
int nch;
vpColVector hiertemp;
//std::string opath = "/local/agpetit/images" + vpIoTools::path("/imagesSoyuz/Hierarchy3/I%04d.pgm");
//std::string opath = "/local/agpetit/images" + vpIoTools::path("/imagesAmazonas/Hierarchy/I%04d.pgm");
std::string opath = "/local/agpetit/images" + vpIoTools::path("/imagesA380/hierarchy1/I%04d.pgm");
char buf0[FILENAME_MAX];
while (lm>0 && argDTTemp.size()>4)
{
	hiertemp.resize(argDTTemp.size());
	ntemp=0;
	threshold = threshold+1;
	//bool inc[argDTTemp.size()];
	inc.resize(argDTTemp.size(),true);
	inc0.resize(argDTTemp.size(),true);
cout << " lm " <<  lm << endl;
	lm=0;
npl=1;
//int mm=0;
		for (int n = 0;n<argDTTemp.size()-2;n++)
	{
			//inc.resize(argDTTemp.size(),true);
			//if (inc0[n+1]==0)
			//{

			cout << " ooooooookkk5 "  <<  endl;
		I0=argDTTemp[n+1];
        //mm++;
		nm=0;
		indice=0;
       simax=threshold;
       nch =0;
       flag =0;
		while (flag ==0)
		{
			flag = 1;
			indice = 0;
			simax = threshold;
			cout << " ooooooookkk2 "  <<  endl;
		for (int k=n+1 ; k<argDTTemp.size()-1;k++)
		{
			if(inc[k+1]==0)
			{
			I1=argDTTemp[k+1];
			sim = computeSimilarity(I0,I1);

			if (sim<threshold && sim<simax)
			{
				simax=sim;
				indice=k+1;
				cout << " ooooooookkk3 " << sim <<  endl;
				//cout << " ooooooookkk3 " << " " << n+1<< " " << k+1 <<  endl;
			}
			}
		}

        if(indice>0)
        {
        I1=argDTTemp[indice];
		mv=merge(I0,I1);
		I0=mv;
		inc[indice]=1;
		inc0[indice]=1;
        hierarchy[nrow][3+nch] = indice;
        nch++;
		flag = 0;
        }
		}
		//inc0[indice]=1;
		cout << " ooooooookkk4 " <<  endl;
		if (nch>0)
		{
		mergeTemp.push_back(I0);
		/*file << level << " "<< n+1 << " " << indice;
		file << std::endl;*/
		hierarchy[nrow][0] = level;
		hierarchy[nrow][1] = n+1;
		hierarchy[nrow][2] = npl;
		nrow++;
		npl++;
        I.resize(I0->getHeight(),I0->getWidth());
		for (int y=0; y<I0->getHeight();y++)
		{
			for (int x=0;x<I0->getWidth();x++)
			{
		I[y][x]=(*I0)[y][x].A;
			}
		}

		/*if (level>1)
		{downScale(I, level+1-2);}*/

		//sprintf(buf0, opath.c_str(), (level+1)*1000 + n+1);
		sprintf(buf0, opath.c_str(), (level+1)*1000 + lm+1);
		std::string filename0(buf0);
		vpImageIo::writePPM(I, filename0);
		//vpImageIo::writePPM(*I0, filename0);
		nm++;
		lm++;
		}

		//cout << " ooooooookkk3 " << nm <<  endl;
		if (nch==0 && inc0[n+1]==0)
		{

			hiertemp[ntemp]=n+1;
			ntemp++;
		}

	//}

	}

		for (int k=0;k<ntemp;k++)
		{
			int nl = hiertemp[k];
			I1=argDTTemp[nl];

			hierarchy[nrow][0] = level;
			hierarchy[nrow][1] = nl;
			hierarchy[nrow][3] = 0;
			hierarchy[nrow][2] = npl;
			nrow++;
			npl++;
           I.resize(I1->getHeight(), I1->getWidth());

			for (int y=0; y<I1->getHeight();y++)
						{
							for (int x=0;x<I1->getWidth();x++)
							{
						I[y][x]=(*I1)[y][x].A;
							}
						}
			             /*if (level>1)
			           {downScale(I, level+1-2);}*/

						sprintf(buf0, opath.c_str(), (level+1)*1000 + lm+1+k);
						std::string filename1(buf0);
						vpImageIo::writePPM(I, filename1);
						//vpImageIo::writePPM(*I1, filename1);



		}


	for (int i = 0; i < argDTTemp.size(); i += 1){
			        I4 = argDTTemp[i];
			        if (I4!=NULL) delete I4;
			        I4=NULL;
			 }
	    argDTTemp.resize(1);
	    //argDTTemp=mergeTemp;
	   cout << " ooooooookkk1 " << mergeTemp.size() << " ntemp " << ntemp << endl;
	  for (int i = 0; i < mergeTemp.size()-1; i += 1){
	  //for (int i = 0; i < floor(mergeTemp.size()/10); i += 1){
		  I3=new vpImage<vpRGBa>;
		  I3->resize(480,640);
		  *I3=*mergeTemp[i+1];
		  argDTTemp.push_back(I3);
	  }

	 for (int i = 0; i < mergeTemp.size(); i += 1){
		  //I3=new vpImage<vpRGBa>;
		  //I3->resize(480,640);
	        mv0 = mergeTemp[i];
		  //*I3=*mv0;
		 //argDTTemp.push_back(I2);
		  //argDTTemp.push_back(mv0);
	        if (mv0!=NULL) delete mv0;
	        mv0=NULL;
		  //delete I2;
	 }
	  mergeTemp.resize(1);
	  I5=argDTTemp[3];
	  cout << " ooooooookkk2 " <<  argDTTemp.size() << endl;
	  /*for (int y=0; y<I5->getHeight();y++)
	  {
	  	for (int x=0;x<I5->getWidth();x++)
	  	{
	  		//cout << " ooooooookkk " << (double)(*I3)[y][x].A<<endl;
	  I[y][x]=(*I5)[y][x].A;
	  	}
	  }*/
	  //vpImageIo::writePPM(I, "merge1.pgm");

	  /*for (int n = 0;n<argDTTemp.size()-1;n++)
	  {
	  I0=argDTTemp[n+1];
	  for (int y=0; y<I0->getHeight();y++)
	  {
	  	for (int x=0;x<I0->getWidth();x++)
	  	{
	  I[y][x]=(*I0)[y][x].A;
	  	}
	  }

	  char buf[FILENAME_MAX];
	  sprintf(buf, opath.c_str(), n);
	  std::string filename(buf);
	  vpImageIo::writePPM(I, filename);

	  }*/

level++;
}
cout << " okkok" << endl;
hierarchy.resize(nrow,40,false);
hierarchy.saveMatrix(filename,hierarchy,false);
//file.close();
/*for (int n = 0;n<argDTTemp.size()-1;n++)
{
I0=argDTTemp[n+1];
for (int y=0; y<I0->getHeight();y++)
{
	for (int x=0;x<I0->getWidth();x++)
	{
I[y][x]=(*I0)[y][x].A;
	}
}

char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), n);
std::string filename0(buf);
vpImageIo::writePPM(I, filename0);

}*/


}

vpImage<vpRGBa>* apViewGeneration::downScale(int i, vpImage<vpRGBa> *_I)
{
//vpImageIo::writePPM(*_I, "Iin.pgm");
unsigned int cScale = static_cast<unsigned int>(pow(2., (int)(i)));
//vpImage<vpRGBa>* I = new vpImage<vpRGBa>(_I->getHeight() / cScale, _I->getWidth() / cScale);
vpImage<vpRGBa>* I = new vpImage<vpRGBa>(_I->getHeight()/2, _I->getWidth()/2);
/*IplImage* vpI0 = cvCreateImageHeader(cvSize(_I->getWidth(), _I->getHeight()), IPL_DEPTH_8U, 4);
vpI0->imageData = (char*)(_I->bitmap);
IplImage* vpI = cvCreateImage(cvSize(_I->getWidth() / cScale, _I->getHeight() / cScale), IPL_DEPTH_8U, 4);
cvResize(vpI0, vpI, CV_INTER_NN);
vpImageConvert::convert(vpI, *I);
cvReleaseImage(&vpI);
vpI0->imageData = NULL;
cvReleaseImageHeader(&vpI0);
vpImage<unsigned char> I1(I->getHeight(),I->getWidth());
for (int y=0; y<I->getHeight();y++)
			{
				for (int x=0;x<I->getWidth();x++)
				{
			I1[y][x]=(*I)[y][x].A;
				}
			}*/

for (unsigned int k = 0, ii = 0; k < I->getHeight(); k += 1, ii += 2){
  for (unsigned int l = 0, jj = 0; l < I->getWidth(); l += 1, jj += 2){
	  //std::cout<< " ookii " << ii <<" ookjj " << jj << std::endl;
    (*I)[k][l].A = (*_I)[ii][jj].A;
    (*I)[k][l].R = (int)(((*_I)[ii][jj].R)/2);
    (*I)[k][l].G = (int)(((*_I)[ii][jj].G)/2);
    (*I)[k][l].B = (int)(((*_I)[ii][jj].B)/2);
  }
}

return I;
}

void apViewGeneration::downScale(vpImage<unsigned char> &_I, int i)
{
//vpImageIo::writePPM(*_I, "Iin.pgm");
unsigned int cScale = static_cast<unsigned int>(pow(2., (int)(i)));
//vpImage<vpRGBa>* I = new vpImage<vpRGBa>(_I->getHeight() / cScale, _I->getWidth() / cScale);
vpImage<unsigned char> I0(_I.getHeight()/cScale, _I.getWidth()/cScale);

for (unsigned int k = 0, ii = 0; k < I0.getHeight(); k += 1, ii += cScale){
  for (unsigned int l = 0, jj = 0; l < I0.getWidth(); l += 1, jj += cScale){
	  //std::cout<< " ookii " << ii <<" ookjj " << jj << std::endl;
    I0[k][l] = _I[ii][jj];
  }
}
_I.resize(I0.getHeight(), I0.getWidth());
_I=I0;
std::cout << " ok "<< std::endl;
//vpImageIo::writePPM((*I), "Iout.pgm");
}

void apViewGeneration::buildHierarchyMS(double threshold, const char *filename)
{
vpImage<vpRGBa> *I0;
vpImage<vpRGBa> *I1;
vpImage<vpRGBa> *I2;
vpImage<vpRGBa> *I3;
vpImage<vpRGBa> *I4;
vpImage<vpRGBa> *I5;
vpImage<vpRGBa> *I6;
vpImage<vpRGBa> *mv;
vpImage<vpRGBa> *mv0;

vpImage<unsigned char> I(480,640);
int nm,lm;
lm=1;
int flag = 0;
int indice;
double sim;
double simax;
vpMatrix hierarchy(10000,40);
vpColVector inc;
vpColVector inc0;
int level=0;
int nrow=0;
int npl,ntemp;
int nch;
vpColVector hiertemp;
//threshold = threshold+1;
std::string opath = "/local/agpetit/images" + vpIoTools::path("/imagesSoyuz/Hierarchy3/I%04d.pgm");
char buf0[FILENAME_MAX];
while (lm>0 && argDTTemp.size()>4)
{
	hiertemp.resize(argDTTemp.size());
	ntemp=0;
	threshold = threshold+1;
	//bool inc[argDTTemp.size()];
	inc.resize(argDTTemp.size(),true);
	inc0.resize(argDTTemp.size(),true);
cout << " lm " <<  lm << endl;
	lm=0;
npl=1;
//int mm=0;
		for (int n = 0;n<argDTTemp.size()-2;n++)
	{
			//inc.resize(argDTTemp.size(),true);
			//if (inc0[n+1]==0)
			//{
			cout << " ooooooookkk5 "  <<  endl;
		I0=argDTTemp[n+1];
        //mm++;
		nm=0;
		indice=0;
       simax=threshold;
       nch=0;
       flag=0;
		while (flag ==0)
		{
			flag = 1;
			indice = 0;
			simax = threshold;
			cout << " ooooooookkk2 "  <<  endl;
		for (int k=n+1 ; k<argDTTemp.size()-1;k++)
		{
			if(inc[k+1]==0)
			{
			I1=argDTTemp[k+1];
			//sim = computeSimilarityScale(I0,I1,level);
			sim = computeSimilarity(I0,I1);

			if (sim<threshold && sim<simax)
			{
				simax=sim;
				indice=k+1;
				cout << " ooooooookkk3 " << sim <<  endl;
				//cout << " ooooooookkk3 " << " " << n+1<< " " << k+1 <<  endl;
			}
			}
		}

        if(indice>0)
        {
        I1=argDTTemp[indice];
		//mv=mergeScale(I0,I1,level);
        mv=merge(I0,I1);
		I0=mv;
		inc[indice]=1;
		inc0[indice]=1;
        hierarchy[nrow][3+nch] = indice;
        nch++;
		flag = 0;
        }
		}
		//inc0[indice]=1;
		if (nch>0)
		{
		//I4=downScale(level+1,I0);
		//mergeTemp.push_back(I4);
		mergeTemp.push_back(I0);
		/*file << level << " "<< n+1 << " " << indice;
		file << std::endl;*/
		hierarchy[nrow][0] = level;
		hierarchy[nrow][1] = n+1;
		hierarchy[nrow][2] = npl;
		nrow++;
		npl++;
		I.resize(I0->getHeight(),I0->getWidth());

		for (int y=0; y<I0->getHeight();y++)
		{
			for (int x=0;x<I0->getWidth();x++)
			{
		I[y][x]=(*I0)[y][x].A;
			}
		}

		downScale(I,level+1);
		//sprintf(buf0, opath.c_str(), (level+1)*1000 + n+1);
		sprintf(buf0, opath.c_str(), (level+1)*1000 + lm+1);
		std::string filename0(buf0);
		vpImageIo::writePPM(I, filename0);
		nm++;
		lm++;
		}

		//cout << " ooooooookkk3 " << nm <<  endl;
		if (nch==0 && inc0[n+1]==0)
		{

			hiertemp[ntemp]=n+1;
			ntemp++;
		}

	//}

	}

		for (int k=0;k<ntemp;k++)
		{
			int nl = hiertemp[k];
			I1=argDTTemp[nl];

			hierarchy[nrow][0] = level;
			hierarchy[nrow][1] = nl;
			hierarchy[nrow][3] = 0;
			hierarchy[nrow][2] = npl;
			nrow++;
			npl++;


			for (int y=0; y<I1->getHeight();y++)
						{
							for (int x=0;x<I1->getWidth();x++)
							{
						I[y][x]=(*I1)[y][x].A;
							}
						}

						sprintf(buf0, opath.c_str(), (level+1)*1000 + lm+1+k);
						std::string filename1(buf0);
						vpImageIo::writePPM(I, filename1);



		}


	for (int i = 0; i < argDTTemp.size(); i += 1){
			        I4 = argDTTemp[i];
			        if (I4!=NULL) delete I4;
			        I4=NULL;
			 }
	    argDTTemp.resize(1);
	    //argDTTemp=mergeTemp;
	   cout << " ooooooookkk1 " << mergeTemp.size() << " ntemp " << ntemp << endl;
	  for (int i = 0; i < mergeTemp.size()-1; i += 1){
	  //for (int i = 0; i < floor(mergeTemp.size()/10); i += 1){
		  I3=new vpImage<vpRGBa>;
		  I3->resize(480,640);
		  *I3=*mergeTemp[i+1];
		  argDTTemp.push_back(I3);
	  }

	 for (int i = 0; i < mergeTemp.size(); i += 1){
		  //I3=new vpImage<vpRGBa>;
		  //I3->resize(480,640);
	        mv0 = mergeTemp[i];
		  //*I3=*mv0;
		 //argDTTemp.push_back(I2);
		  //argDTTemp.push_back(mv0);
	        if (mv0!=NULL) delete mv0;
	        mv0=NULL;
		  //delete I2;
	 }
	  mergeTemp.resize(1);
	  I5=argDTTemp[3];
	  cout << " ooooooookkk2 " <<  argDTTemp.size() << endl;
	  /*for (int y=0; y<I5->getHeight();y++)
	  {
	  	for (int x=0;x<I5->getWidth();x++)
	  	{
	  		//cout << " ooooooookkk " << (double)(*I3)[y][x].A<<endl;
	  I[y][x]=(*I5)[y][x].A;
	  	}
	  }*/
	  //vpImageIo::writePPM(I, "merge1.pgm");

	  /*for (int n = 0;n<argDTTemp.size()-1;n++)
	  {
	  I0=argDTTemp[n+1];
	  for (int y=0; y<I0->getHeight();y++)
	  {
	  	for (int x=0;x<I0->getWidth();x++)
	  	{
	  I[y][x]=(*I0)[y][x].A;
	  	}
	  }

	  char buf[FILENAME_MAX];
	  sprintf(buf, opath.c_str(), n);
	  std::string filename(buf);
	  vpImageIo::writePPM(I, filename);

	  }*/

level++;
}
cout << " okkok" << endl;
hierarchy.resize(nrow,40,false);
hierarchy.saveMatrix(filename,hierarchy,false);
//file.close();
/*for (int n = 0;n<argDTTemp.size()-1;n++)
{
I0=argDTTemp[n+1];
for (int y=0; y<I0->getHeight();y++)
{
	for (int x=0;x<I0->getWidth();x++)
	{
I[y][x]=(*I0)[y][x].A;
	}
}

char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), n);
std::string filename0(buf);
vpImageIo::writePPM(I, filename0);

}*/


}


double apViewGeneration::computeSimilarity(vpImage<vpRGBa> *I0, vpImage<vpRGBa> *I1)
{

int width = I0->getWidth();
int height = I0->getHeight();
int m,mo,yy,xx,adtIy,adtIx;
double cor,t0,t1,dist,delta,oriT,oriI,dtI;
int offs =127;

t0= vpTime::measureTimeMs();
dist=0;
m=0;
mo=0;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {

	  if (((*I0)[y][x].A<100 || (*I0)[y][x].A>100) && y-(((*I1)[y][x]).R-offs)*2>0 && y-(((*I1)[y][x]).R-offs)*2<480 &&  x-(((*I1)[y][x]).G-offs)*2>0 &&  x-(((*I1)[y][x]).G-offs)*2<640)
    {
    	m++;
    	//cout << " ok " << y-(((*I1)[y][x]).R-offs)*2 << " ok0 " <<  x-(((*I1)[y][x]).G-offs)*2 << endl;
    	//oriI = (double)(*Im)[adtIy][adtIx];
    	oriI = (double)(*I1)[y-(((*I1)[y][x]).R-offs)*2][x-(((*I1)[y][x]).G-offs)*2].A;
    	yy=-1;
    	xx=-1;
    	while (oriI==100 && yy<2)
    		{
    		oriI=(double)(*I1)[y-(((*I1)[y][x]).R-offs)*2+yy][x-(((*I1)[y][x]).G-offs)*2 + xx].A;
    		yy ++;
			while ( oriI==100 && xx<2)
			{
				oriI=(double)(*I1)[y-(((*I1)[y][x]).R-offs)*2+yy][x-(((*I1)[y][x]).G-offs)*2 + xx].A;
				xx ++;
			}
    		}
    	(*I1)[y-(((*I1)[y][x]).R-offs)*2][x-(((*I1)[y][x]).G-offs)*2].A= oriI;
    	if (oriI>100 || oriI<100)
    		{
    		mo++;
    		delta=abs((*I0)[y][x].A-oriI);
    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
    		if (delta>255-delta) delta=255-delta;
	   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
    		//cout  <<" " << delta << endl;
    	//dist = dist + dtI + 0.1*delta;
    		dist = dist + (double)(*I1)[y][x].B + 0.1*delta;
    	//dist = dist  + 0.1*delta;
    		}
    	/*else
    	dist = dist + (ImDT)[y][x];*/
    }
  }
}

//cout << dist/mo << endl;
dist=dist/mo;
t1= vpTime::measureTimeMs();
//cout << " timem " << t1-t0 << endl;

return dist;
}

double apViewGeneration::computeSimilarityScale(vpImage<vpRGBa> *I0, vpImage<vpRGBa> *I1, int scale)
{

int width = I0->getWidth();
int height = I0->getHeight();
int m,mo,yy,xx,adtIy,adtIx;
double cor,t0,t1,dist,delta,oriT,oriI,dtI;
unsigned int cScale = static_cast<unsigned int>(pow(2., (int)(scale)));
int offs = (int)(127/cScale);
//std::cout << "offs " << offs << std::endl;
t0= vpTime::measureTimeMs();
dist=0;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {

	  if ((*I0)[y][x].A<100 || (*I0)[y][x].A>100)
    {
    	m++;
    	//oriI = (double)(*Im)[adtIy][adtIx];
    	oriI = (double)(*I1)[y-(((*I1)[y][x]).R-offs)*2][x-(((*I1)[y][x]).G-offs)*2].A;
    	yy=-1;
    	xx=-1;
    	while (oriI==100 && yy<2)
    		{
    		oriI=(double)(*I1)[y-(((*I1)[y][x]).R-offs)*2+yy][x-(((*I1)[y][x]).G-offs)*2 + xx].A;
    		yy ++;
			while ( oriI==100 && xx<2)
			{
				oriI=(double)(*I1)[y-(((*I1)[y][x]).R-offs)*2+yy][x-(((*I1)[y][x]).G-offs)*2 + xx].A;
				xx ++;
			}
    		}
    	(*I1)[y-(((*I1)[y][x]).R-offs)*2][x-(((*I1)[y][x]).G-offs)*2].A= oriI;
    	if (oriI>100 || oriI<100)
    		{
    		mo++;
    		delta=abs((*I0)[y][x].A-oriI);
    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
    		if (delta>255-delta) delta=255-delta;
	   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
    		//cout  <<" " << delta << endl;
    	//dist = dist + dtI + 0.1*delta;
    		dist = dist + (double)(*I1)[y][x].B + 0.05*delta;
    	//dist = dist  + 0.1*delta;
    		}
    	/*else
    	dist = dist + (ImDT)[y][x];*/
    }
  }
}

//cout << dist/mo << endl;
dist=dist/mo;
t1= vpTime::measureTimeMs();
//cout << " timem " << t1-t0 << endl;

return dist;
}

void apViewGeneration::computeSimilarityMatrix(const char *filename){

vpMatrix hierarchy(300000,3);
vpImage<vpRGBa> *I0;
vpImage<vpRGBa> *I1;
double sim;
int kk=0;
	for (int n = 0;n<argDTTemp.size()-2;n++)
{

	I0=argDTTemp[n+1];
    //mm++;
	for (int k=0 ; k<argDTTemp.size()-1;k++)
	{
		if(k!=n){
		I1=argDTTemp[k+1];
		sim = computeSimilarity(I0,I1);
		hierarchy[kk][0] = n+1;
		hierarchy[kk][1] = k+1;
		hierarchy[kk][2] = sim;
		kk++;
		}
	}
}
hierarchy.resize(kk,3,false);
hierarchy.saveMatrix(filename,hierarchy,false);
for (int i = 0; i < argDTTemp.size(); i += 1){
		        I1 = argDTTemp[i];
		        if (I1!=NULL) delete I1;
		        I1=NULL;
		 }
}

double apViewGeneration::computeSimilarity(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT)
{
int width = Im->getWidth();
int height = Im->getHeight();
int m,mo,yy,xx,adtIy,adtIx;
double cor,t0,t1,dist,delta,oriT,oriI,dtI;
int offs =(int) 127*((double)width/640);
//int offs =(int) 127;
t0= vpTime::measureTimeMs();
dist=0;
mo=0;
m=0;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {

	  if ((*IT)[y][x]<100 || (*IT)[y][x]>100)
    {
		  //if ((double)(*Im)[y][x].B<255)


    	m++;
    	//oriI = (double)(*Im)[adtIy][adtIx];
    	oriI = (double)(*Im)[y-(((*Im)[y][x]).R-offs)*2][x-(((*Im)[y][x]).G-offs)*2].A;
    	yy=-1;
    	xx=-1;
    	while (oriI==100 && yy<2)
    		{
    		oriI=(double)(*Im)[y-(((*Im)[y][x]).R-offs)*2+yy][x-(((*Im)[y][x]).G-offs)*2 + xx].A;
    		yy ++;
			while ( oriI==100 && xx<2)
			{
				oriI=(double)(*Im)[y-(((*Im)[y][x]).R-offs)*2+yy][x-(((*Im)[y][x]).G-offs)*2 + xx].A;
				xx ++;
			}
    		}
    	(*Im)[y-(((*Im)[y][x]).R-offs)*2][x-(((*Im)[y][x]).G-offs)*2].A = oriI;
    	if (oriI>100 || oriI<100)
    		{
    		mo++;
    		delta=abs((*IT)[y][x]-oriI);
    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
    		if (delta>255-delta) delta=255-delta;
	   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
    		//cout  <<" " << delta << endl;
    	//dist = dist + dtI + 0.1*delta;
    		dist = dist + (double)(*Im)[y][x].B + 0.3*delta;
    		//cout << mo << endl;
    	//dist = dist  + 0.1*delta;
    		}
    	/*else
    	dist = dist + (ImDT)[y][x];*/

    }
  }
}

dist=dist/mo;
t1= vpTime::measureTimeMs();
//cout << " timem " << t1-t0 << endl;

return dist;
}


void apViewGeneration::detect(vpImage<unsigned char> &I0, std::string opath)
{

edgeOrientMap(I0);
//vpImageIo::writePPM(I0, "edgeim.pgm");
vpImage<vpRGBa> *argImDT;
argImDT=dt0(&I0);
const char *filename = "hierarchy.txt";
	std::fstream file;
	//if (!binary)creat
//file.open(filename, std::fstream::in | std::fstream::ate);
//file.open(filename, std::fstream::in);
vpImage<unsigned char> IT0;
vpImage<unsigned char> IT1;
int level;
int im1;
int im2;
int pos;
int npl;
string ap;
vpMatrix hierarchy;
hierarchy.loadMatrix(filename,hierarchy,false);
cout << hierarchy << endl;
//getchar();
//file.seekg(0,fstream::end);
//int length = (long)file.tellg();
//pos=(int)length-2;
int size;
int count=0;
double sim,sim0,sim1;
double simmax=100;
vpImage<unsigned char> IT(480,640);

int i=hierarchy.getRows()-1;

	level = hierarchy[i][0]+1;
	int j=1;
	int ind=0;
	int m;
	std::vector<int> underT;



	while(hierarchy[i-j+1][0]==level-1){
	//while(j<64){
		char buf[FILENAME_MAX];
		sprintf(buf, opath.c_str(),level*1000+j);
		//sprintf(buf, opath.c_str(),j-1);
		std::string filename(buf);
		//std::cout << "Write: " << filename << std::endl;
		vpImageIo::readPPM(IT, filename);
		sim = computeSimilarity(argImDT,&IT);
		count++;
		cout << " simi "<< sim << " " << j << endl;
if(sim<simmax)
{
	simmax=sim;
	ind=j;
}
j++;
	}
	//getchar();
	//cout << " okd "<< ind << " " << i << " " << j << endl;
im1=hierarchy[i-j+ind+1][1];
im2= hierarchy[i-j+ind+1][2];
/*if (im2==0)
{
	im2=im1;
}*/
i-=j-1;
cout << " okd " << im1 << " okd1 " << im2 << endl;
simmax=100;
ind=0;


	while (i>-1)
	{

level = hierarchy[i][0]+1;
npl = hierarchy[i][3];


/*if (underT.size()>0)
{
	simmax=100;
for (int l=0 ; l<underT.size() ; l++)
{

	int indu=underT[l];

	double sim = underT[l];

	        char buf[FILENAME_MAX];
			sprintf(buf, opath.c_str(),level*1000+indu);
			std::string filename(buf);
			//std::cout << "Write: " << filename << std::endl;
			vpImageIo::readPPM(IT0, filename);
			sim = computeSimilarity(argImDT,&IT0);
			count++;
			if (sim<simmax)
			{
				simmax=sim;
				ind=indu;
			}

			cout << " level "<< level << " indu " << indu << " sim " << sim <<  endl;
}
}*/

cout << " level " << level << " ind "<< ind << "simmax " << simmax <<  endl;


/*if (im2>0)
{*/
char buf0[FILENAME_MAX];
		sprintf(buf0, opath.c_str(),level*1000+im1);
		std::string filename0(buf0);
		//std::cout << "Write: " << filename << std::endl;
		vpImageIo::readPPM(IT0, filename0);
		sim0 = computeSimilarity(argImDT,&IT0);
		count++;
char buf1[FILENAME_MAX];
		sprintf(buf1, opath.c_str(),level*1000+im2);
		std::string filename1(buf1);
				//std::cout << "Write: " << filename << std::endl;
		vpImageIo::readPPM(IT1, filename1);
		sim1 = computeSimilarity(argImDT,&IT1);
		count++;

		if (sim0>simmax && sim1>simmax)
		{
			im1=hierarchy[i-npl+ind][1];
			im2=hierarchy[i-npl+ind][2];
		}

		else if (sim0<sim1)
		{
			im2=hierarchy[i-npl+im1][2];
			im1=hierarchy[i-npl+im1][1];

		}
		else if (sim1<sim0)
		{
			im1=hierarchy[i-npl+im2][1];
			im2=hierarchy[i-npl+im2][2];
		}

		cout << "sim0 " << sim0 << " sim1 "<< sim1 << " im1 "<< im1 << " im2 " << im2 <<  endl;

		//}
		/*else
		{
			im1=hierarchy[i-npl+im1][1];
			im2=hierarchy[i-npl+im1][2];
		}*/
		//underT.resize(0);
        m=0;
        simmax=100;
		while (hierarchy[i-m][2]==0)
		{
           //int indi=hierarchy[i-m][1];

			char buf[FILENAME_MAX];
			//sprintf(buf, opath.c_str(),level*1000+indi);
			sprintf(buf, opath.c_str(),level*1000+npl-m);
			std::string filename(buf);
						//std::cout << "Write: " << filename << std::endl;
			vpImageIo::readPPM(IT0, filename);
			sim = computeSimilarity(argImDT,&IT0);
			count++;
			cout << "sim " << sim <<  endl;
			if (sim<simmax)
				{
					simmax=sim;
					ind=hierarchy[i-m][1];
				}


			//underT.push_back(hierarchy[i-m][1]);
			m++;
		}

		//cout << " im1 " << im1 << " im2 " << im2 << " sim0 "<< sim0 << " sim1 " << sim1 << " count " << count << endl;
i-=npl;
}
}

void apViewGeneration::detect1(vpImage<unsigned char> &I0, std::string opath)
	{

	edgeOrientMap(I0);
	vpImage<unsigned char> I1(480,640);
	//vpImageIo::writePPM(I0, "inputedge.pgm");
	vpImage<vpRGBa> *argImDT;
	//vpImage<vpRGBa> *argImDT0;
	//vpImage<vpRGBa> *argImDT = new vpImage<vpRGBa>(480,640);
	//vpImage<vpRGBa> *argImDT0 = new vpImage<vpRGBa>(480,640);
	argImDT=dt0(&I0);
	const char *filename = "hierarchySat.txt";
		std::fstream file;
		  for (int x = 0; x < 640; x++) {
		    for (int y = 0; y < 480; y++) {
		      I1[y][x] = (*argImDT)[y][x].B;
		    }
		  }
    vpImageIo::writePPM(I1, "DTI.pgm");
	  for (int x = 0; x < 640; x++) {
	    for (int y = 0; y < 480; y++) {
	    	if(I0[y][x]!=100)
	    	{
	      I1[y][x] = 0;
	    	}
	    	else
	    	{
	  	      I1[y][x] = 255;
	    	}
	    }
	  }
vpImageIo::writePPM(I1, "EdgeM.pgm");
	vpImage<unsigned char> IT0;
	vpImage<unsigned char> IT1;
	int level;
	int im1;
	int im2;
	int pos;
	int npl;
	int kmax;
	vpColVector im(20);
	string ap;
	vpMatrix hierarchy;
	hierarchy.loadMatrix(filename,hierarchy,false);
	cout << hierarchy << endl;

	int size;
	int count=0;
	double sim,sim0,sim1;
	double simmax=1000;
	vpImage<unsigned char> IT(480,640);

	int i=hierarchy.getRows()-1;

		level = hierarchy[i][0]+1;
		cout << " level " << level << endl;
		int j=1;
		int ind=0;
		int m;
		std::vector<int> underT;

		/*for (int k=0;k<441;k++)
		{
			char buf[FILENAME_MAX];
			sprintf(buf, opath.c_str(),k);
			//sprintf(buf, opath.c_str(),j-1);
			std::string filename(buf);
			//std::cout << "Write: " << filename << std::endl;
			vpImageIo::readPPM(IT, filename);
			//vpImageIo::writePPM(IT, "it0.pgm");
			sim = computeSimilarity(argImDT,&IT);
			cout << " sim "<< k << " " << sim << endl;
		}
		getchar();*/

		while(hierarchy[i-j+1][0]==level-1){
		//while(j<64){
			char buf[FILENAME_MAX];
			sprintf(buf, opath.c_str(),level*1000+j);
			//sprintf(buf, opath.c_str(),j-1);
			std::string filename(buf);
			//std::cout << "Write: " << filename << std::endl;
			vpImageIo::readPPM(IT, filename);
			//vpImageIo::readPPM(*argImDT, filename);
			double t0= vpTime::measureTimeMs();
			sim = computeSimilarity(argImDT,&IT);
			double t1= vpTime::measureTimeMs();
			//sim = computeSimilarity(argImDT,&I0);
			count++;
			cout << "Time simi "<< t1-t0 << endl;
			cout << " simi "<< sim << " " << j << endl;
	if(sim<simmax)
	{
		simmax=sim;
		ind=j;
	}
	j++;
		}
		//getchar();
		cout << " okd "<< ind << " " << i << " " << j << endl;
		int arb =0;
		if(hierarchy[i-j+ind+1][3]>0)
		{
	 while (hierarchy[i-j+ind+1][3+arb]>0)
	 {
	im[0]=hierarchy[i-j+ind+1][1];
		 //im[0]=8;
	im[arb+1]= hierarchy[i-j+ind+1][3+arb];
	arb++;
	 }
		}
		else {
			im[0]=hierarchy[i-j+ind+1][1];
		}
	/*if (im2==0)
	{
		im2=im1;
	}*/
	i-=j-1;
	cout << " okd " << im[0] << " okd1 " << im[1] << endl;
	simmax=1000;
	ind=0;

		while (i>-1)
		{

	level = hierarchy[i][0]+1;
	npl = hierarchy[i][2];


	cout << " level " << level << " ind "<< ind << "simmax " << simmax <<  endl;


	/*if (im2>0)
	{*/
	int k =0;
	int flag = 0;
while(im[k]>0)
{
	char buf0[FILENAME_MAX];
	int nim = level*1000+im[k];
			sprintf(buf0, opath.c_str(),nim);
			std::string filename0(buf0);
			vpImageIo::readPPM(IT0, filename0);
			sim0 = computeSimilarity(argImDT,&IT0);
			//vpImageIo::readPPM(*argImDT0, filename0);
			//sim = computeSimilarity(argImDT,&IT);
			//sim = computeSimilarity(argImDT0,&I0);
			count++;
			std::cout << " simmax " << simmax << std::endl;
			if(sim0<simmax)
			{
				kmax = k;
				simmax = sim0;
				flag = 1;
			}
			std::cout << "sim0 " << sim0 << std::endl;
k++;
}
int imax = im[kmax];
std::cout << "imax: " << imax << std::endl;
im.resize(20,true);
arb = 0;
if (flag == 1)
{
while (hierarchy[i-npl+imax][3+arb]>0)
{
	//std::cout << "ind: " <<i-npl+imax << " flag " << flag << std::endl;
im[0]=hierarchy[i-npl+imax][1];
im[arb+1]= hierarchy[i-npl+imax][3+arb];
arb++;
}
}
else {
	while (hierarchy[i-npl+ind][3+arb]>0)
	{
	im[0]=hierarchy[i-npl+ind][1];
	im[arb+1]= hierarchy[i-npl+ind][3+arb];
	arb++;
	}
}

			cout << " ok " << im << endl;

	        m=0;
	        simmax=1000;
			while (hierarchy[i-m][3]==0)
			{
	           //int indi=hierarchy[i-m][1];

				char buf[FILENAME_MAX];
				//sprintf(buf, opath.c_str(),level*1000+indi);
				sprintf(buf, opath.c_str(),level*1000+npl-m);
				std::string filename(buf);
							//std::cout << "Write: " << filename << std::endl;
				vpImageIo::readPPM(IT0, filename);
				sim = computeSimilarity(argImDT,&IT0);
				count++;
				cout << "sim " << sim <<  endl;
				if (sim<simmax)
					{
						simmax=sim;
						ind=hierarchy[i-m][1];
					}


				//underT.push_back(hierarchy[i-m][1]);
				m++;
			}

			cout << " im " << im << " simmax "<< simmax  << endl;
	i-=npl;
	}




}





void
apViewGeneration::initPyramid(vpImage<vpRGBa>* _I, std::vector< vpImage<vpRGBa>* >& _pyramid)
{
  _pyramid.resize(7);

  /*if(scales[0]){
    _pyramid[0] = &_I;
  }
  else{
    _pyramid[0] = NULL;
  }*/

  for(unsigned int i=1; i<_pyramid.size(); i += 1){
    //if(scales[i]){
      unsigned int cScale = static_cast<unsigned int>(pow(2., (int)i-1));
      vpImage<vpRGBa>* I = new vpImage<vpRGBa>(_I->getHeight() / cScale, _I->getWidth() / cScale);

      for (unsigned int k = 0, ii = 0; k < I->getHeight(); k += 1, ii += cScale){
        for (unsigned int l = 0, jj = 0; l < I->getWidth(); l += 1, jj += cScale){
          (*I)[k][l].A = (*_I)[ii][jj].A;
          (*I)[k][l].R = (int)(*_I)[ii][jj].R/cScale;
          (*I)[k][l].G = (int)(*_I)[ii][jj].G/cScale;
          (*I)[k][l].B = (int)(*_I)[ii][jj].B/cScale;

        }
      }
      _pyramid[i] = I;
    }
    /*else{
      _pyramid[i] = NULL;
    }*/
  //}
}

void apViewGeneration::detectMS(vpImage<unsigned char> &I0, std::string opath)
	{
	//std::vector< const vpImage<vpRGBa>* >& Ipyramid;
	//Ipyramid.resize(0);
	edgeOrientMap(I0);
	vpImage<unsigned char> I1(480,640);
	vpImageIo::writePPM(I0, "inputedge.pgm");
	vpImage<vpRGBa> *argImDT;
	vpImage<vpRGBa> *argImDTP;
	argImDT=dt0(&I0);
	initPyramid(argImDT,Ipyramid);

	argImDTP = Ipyramid[3];
	std::cout << " size " << argImDTP->getHeight() << std::endl;
	const char *filename = "hierarchy3.txt";
		std::fstream file;
		I1.resize(argImDTP->getHeight(),argImDTP->getWidth());
		  for (int x = 0; x < argImDTP->getWidth(); x++) {
		    for (int y = 0; y < argImDTP->getHeight(); y++) {
		      I1[y][x] = (*argImDTP)[y][x].B;
		    }
		  }
    vpImageIo::writePPM(I1, "DTI.pgm");
    /*I0.resize(argImDTP->getHeight(),argImDTP->getWidth());
	  for (int x = 0; argImDTP->getWidth(); x++) {
	    for (int y = 0; y < argImDTP->getHeight(); y++) {
	    	if(I0[y][x]!=100)
	    	{
	      I1[y][x] = 0;
	    	}
	    	else
	    	{
	  	      I1[y][x] = 255;
	    	}
	    }
	  }
vpImageIo::writePPM(I1, "EdgeM.pgm");*/
	vpImage<unsigned char> IT0;
	vpImage<unsigned char> IT1;
	int level;
	int im1;
	int im2;
	int pos;
	int npl;
	int kmax;
	vpColVector im(20);
	string ap;
	vpMatrix hierarchy;
	hierarchy.loadMatrix(filename,hierarchy,false);
	cout << hierarchy << endl;

	int size;
	int count=0;
	double sim,sim0,sim1;
	double simmax=100;
	vpImage<unsigned char> IT;

	int i=hierarchy.getRows()-1;

		level = hierarchy[i][0]+1;
		cout << " level " << level << endl;
		int j=1;
		int ind=0;
		int m;
		std::vector<int> underT;
		if (level>1)
		argImDTP = Ipyramid[level+1-1];
		else
		argImDTP = Ipyramid[1];
		while(hierarchy[i-j+1][0]==level-1){
		//while(j<64){
			char buf[FILENAME_MAX];
			sprintf(buf, opath.c_str(),level*1000+j);
			//sprintf(buf, opath.c_str(),j-1);
			std::string filename(buf);
			//std::cout << "Write: " << filename << std::endl;
			vpImageIo::readPPM(IT, filename);
			cout << " level0 " << argImDTP->getHeight() << " level1 " << IT.getHeight() << endl;
			sim = computeSimilarity(argImDTP,&IT);
			count++;
			cout << " simi "<< sim << " " << j << endl;
	if(sim<simmax)
	{
		simmax=sim;
		ind=j;
	}
	j++;
		}
		//getchar();
		cout << " okd "<< ind << " " << i << " " << j << endl;
		int arb =0;
		if(hierarchy[i-j+ind+1][3]>0)
		{
	 while (hierarchy[i-j+ind+1][3+arb]>0)
	 {
	im[0]=hierarchy[i-j+ind+1][1];
		 //im[0]=8;
	im[arb+1]= hierarchy[i-j+ind+1][3+arb];
	arb++;
	 }
		}
		else {
			im[0]=hierarchy[i-j+ind+1][1];
		}
	/*if (im2==0)
	{
		im2=im1;
	}*/
	i-=j-1;
	cout << " okd " << im[0] << " okd1 " << im[1] << endl;
	simmax=100;
	ind=0;

		while (i>-1)
		{

	level = hierarchy[i][0]+1;
	if (level>1)
	{argImDTP = Ipyramid[level+1-1];}
	else
	{argImDTP = Ipyramid[1];}

	npl = hierarchy[i][2];


	cout << " level " << level << " ind "<< ind << "simmax " << simmax <<  endl;


	/*if (im2>0)
	{*/
	int k =0;
	int flag = 0;
while(im[k]>0)
{
	char buf0[FILENAME_MAX];
	int nim = level*1000+im[k];
			sprintf(buf0, opath.c_str(),nim);
			std::string filename0(buf0);
			vpImageIo::readPPM(IT0, filename0);
			sim0 = computeSimilarity(argImDTP,&IT0);
			count++;
			std::cout << "simmax " << simmax << std::endl;
			if(sim0<simmax)
			{
				kmax = k;
				simmax = sim0;
				flag = 1;
			}
			std::cout << "sim0 " << sim0 << std::endl;
k++;
}
int imax = im[kmax];
std::cout << "imax: " << imax << std::endl;
im.resize(20,true);
arb = 0;
if (flag == 1)
{
while (hierarchy[i-npl+imax][3+arb]>0)
{
	//std::cout << "ind: " <<i-npl+imax << " flag " << flag << std::endl;
im[0]=hierarchy[i-npl+imax][1];
im[arb+1]= hierarchy[i-npl+imax][3+arb];
arb++;
}
}
else {
	while (hierarchy[i-npl+ind][3+arb]>0)
	{
	im[0]=hierarchy[i-npl+ind][1];
	im[arb+1]= hierarchy[i-npl+ind][3+arb];
	arb++;
	}
}

			cout << " ok " << im << endl;

	        m=0;
	        simmax=100;
			while (hierarchy[i-m][3]==0)
			{
	           //int indi=hierarchy[i-m][1];

				char buf[FILENAME_MAX];
				//sprintf(buf, opath.c_str(),level*1000+indi);
				sprintf(buf, opath.c_str(),level*1000+npl-m);
				std::string filename(buf);
							//std::cout << "Write: " << filename << std::endl;
				vpImageIo::readPPM(IT0, filename);
				sim = computeSimilarity(argImDTP,&IT0);
				count++;
				cout << "sim " << sim <<  endl;
				if (sim<simmax)
					{
						simmax=sim;
						ind=hierarchy[i-m][1];
					}


				//underT.push_back(hierarchy[i-m][1]);
				m++;
			}

			cout << " im " << im << " simmax "<< simmax  << endl;
	i-=npl;
	}




}
void apViewGeneration::rotateTemp( vpImage<unsigned char> &IT, double angle)
	{
	vpImage<unsigned char> Irot(480,640);
	int height = IT.getHeight();
	int width = IT.getWidth();
	double rho,theta;
	int xrot,yrot;
	for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  (Irot)[y][x]=100;
		  }
	}
	for (int y = 0; y < height; y++) {
	  for (int x = 0; x < width; x++) {

		  if (((IT)[y][x]<100 || (IT)[y][x]>100) && x!=320)
	    {
		rho = sqrt((y-240)*(y-240)+(x-320)*(x-320));
		theta = atan2(-(y-240),(x-320));
		theta = theta+angle;
		xrot = (int)(rho*cos(theta)+320);
		yrot = (int)(-rho*sin(theta)+240);
		  //cout << " xrot " << xrot << " yrot " << yrot << endl;
		if(xrot<640 && xrot>0 && yrot>0 && yrot<480)
		{
			if((IT)[y][x]-255*(angle/M_PI)>0  && (IT)[y][x]-255*(angle/M_PI)<255 )
			{
		(Irot)[yrot][xrot]=(IT)[y][x] - (int)255*(angle/M_PI);
			}
			else if ((IT)[y][x]-255*(angle/M_PI)<0)
			{
		(Irot)[yrot][xrot]=	255 + (IT)[y][x] - (int)255*(angle/M_PI);
			}
			else if ((IT)[y][x]-255*(angle/M_PI)>255)
			{
		(Irot)[yrot][xrot]=	-255 + (IT)[y][x] - 255*(angle/M_PI);
			}
		}
	    }
	  }
	}
	IT=Irot;
	}

void apViewGeneration::detect2(vpImage<unsigned char> &I0, std::string opath)
	{
	//std::vector< const vpImage<vpRGBa>* >& Ipyramid;
	//Ipyramid.resize(0);
	edgeOrientMap(I0);
	vpImage<unsigned char> I1(480,640);
	//vpImageIo::writePPM(I0, "inputedge.pgm");
	vpImage<vpRGBa> *argImDT;
	vpImage<vpRGBa> *argImDTP;
	argImDT=dt0(&I0);
	initPyramid(argImDT,Ipyramid);
	std::vector<vpImagePoint> pointsPos;
	argImDTP = Ipyramid[3];
	std::cout << " size " << argImDTP->getHeight() << std::endl;
	const char *filename = "hierarchyA380.txt";
		std::fstream file;
		I1.resize(argImDTP->getHeight(),argImDTP->getWidth());
		  for (int x = 0; x < argImDTP->getWidth(); x++) {
		    for (int y = 0; y < argImDTP->getHeight(); y++) {
		      I1[y][x] = (*argImDTP)[y][x].B;
		    }
		  }
    //vpImageIo::writePPM(I1, "DTI.pgm");
    /*I0.resize(argImDTP->getHeight(),argImDTP->getWidth());
	  for (int x = 0; argImDTP->getWidth(); x++) {
	    for (int y = 0; y < argImDTP->getHeight(); y++) {
	    	if(I0[y][x]!=100)
	    	{
	      I1[y][x] = 0;
	    	}
	    	else
	    	{
	  	      I1[y][x] = 255;
	    	}
	    }
	  }
vpImageIo::writePPM(I1, "EdgeM.pgm");*/
	vpImage<unsigned char> IT0;
	vpImage<unsigned char> IT1;
	int level;
	int im1;
	int im2;
	int pos;
	int npl;
	int kmax;
	vpColVector im(20);
	string ap;
	vpMatrix hierarchy;
	hierarchy.loadMatrix(filename,hierarchy,false);
	cout << hierarchy << endl;

	int size;
	int count=0;
	double sim,sim0,sim1;
	double simmax=2000;
	vpImage<unsigned char> IT;

	int i=hierarchy.getRows()-1;

		level = hierarchy[i][0]+1;
		//unsigned int cScale = static_cast<unsigned int>(pow(2., (int)(level)));
		vpImagePoint pI;
		int nc = (int)640;
		int nr = (int)480;
		int intc = (int)nc/5;
		int intr = (int)nr/5;
		pointsPos.resize(0);
		double Ang,AngOpt0,AngOpt;
		double rotm=-M_PI;
		double ang;

		cout << " level " << level << endl;
		int j=1;
		int ind=0;
		int m;
		double simmaxP=2000;
		double imoy = 0;
		double jmoy = 0;
		std::vector<int> underT;
		vpImagePoint Popt;
        vpImagePoint Popt0;
        vpImagePoint Po(240,420);
        int kopt;
		/*if (level>1)
		argImDTP = Ipyramid[level+1-1];
		else
		argImDTP = Ipyramid[1];*/

		for (int k=0;k<441;k++)
		{
			char buf[FILENAME_MAX];
			sprintf(buf, opath.c_str(),k);
			//sprintf(buf, opath.c_str(),j-1);
			std::string filename(buf);
			//std::cout << "Write: " << filename << std::endl;
			vpImageIo::readPPM(IT, filename);
			//vpImageIo::writePPM(IT, "it0.pgm");
			for (int cc=0; cc<5;cc++)
			{
				for (int rr=0; rr<5;rr++)
				{
				Po.set_i(rr*intr+(int)intr/2);
				Po.set_j(cc*intc+(int)intc/2);
				for (int rot=0; rot<10;rot++)
				{
					ang=rot*(rotm/5)-rotm;
					rotateTemp(IT,ang);
			sim = computeSimilarityPos(argImDT,&IT,Po);
			cout << " sim "<< k << " " << sim << endl;

			if ((double)sim<simmaxP)
			{
			simmaxP = sim;
			Popt0 = Po;
			AngOpt0=ang;
			kopt =k;
			}
				}
				}
			}
		}
		cout << " simi "<< simmaxP << " " << kopt << " " << Popt0.get_i() << " "<< Popt0.get_j()<< " "<< AngOpt0 << endl;

		getchar();
		while(hierarchy[i-j+1][0]==level-1){
		//while(j<64){
			char buf[FILENAME_MAX];
			sprintf(buf, opath.c_str(),level*1000+j);
			//sprintf(buf, opath.c_str(),j-1);
			std::string filename(buf);
			//std::cout << "Write: " << filename << std::endl;
			vpImageIo::readPPM(IT, filename);

			for (int cc=0; cc<5;cc++)
			{
				for (int rr=0; rr<5;rr++)
				{

				pI.set_i(rr*intr+(int)intr/2);
				pI.set_j(cc*intc+(int)intc/2);
				for (int rot=0; rot<10;rot++)
				{
					ang=rot*(rotm/5)-rotm;
					rotateTemp(IT,ang);
				//pointsPos.push_back(pI);
				//cout << " sim " << endl;
				sim = computeSimilarityPos(argImDT,&IT,pI);
				cout << " simi "<< sim << " " << pI.get_i() << " "<< pI.get_j()<< " "<< ang << endl;
				if ((double)sim<simmaxP)
				{
				simmaxP = sim;
				Popt0 = pI;
				AngOpt0=ang;
				}
				}
				}
			}
				sim=simmaxP;

			count++;
			cout << " simi0 "<< sim << " " << j << " " << Popt.get_i() << " "<< Popt.get_j()<< " "<< AngOpt0 << endl;
	if(sim<simmax)
	{
		Popt = Popt0;
		AngOpt = AngOpt0;
		simmax=sim;
		ind=j;
	}
	j++;
		}
		//getchar();
		//cout << " okd "<< ind << " " << i << " " << j << endl;
		int arb =0;
		if(hierarchy[i-j+ind+1][3]>0)
		{
	 while (hierarchy[i-j+ind+1][3+arb]>0)
	 {
	im[0]=hierarchy[i-j+ind+1][1];
		 //im[0]=8;
	im[arb+1]= hierarchy[i-j+ind+1][3+arb];
	arb++;
	 }
		}
		else {
			im[0]=hierarchy[i-j+ind+1][1];
		}
	/*if (im2==0)
	{
		im2=im1;
	}*/
	i-=j-1;
	//cout << " okd " << im[0] << " okd1 " << im[1] << endl;
	simmax=2000;
	simmaxP=2000;
	ind=0;
	pI=Popt;
	Ang=AngOpt;
	rotm=rotm/2;

		while (i>-1)
		{

	level = hierarchy[i][0]+1;

	if (level>1)
	{argImDTP = Ipyramid[level+1-1];}
	else
	{argImDTP = Ipyramid[1];}

	npl = hierarchy[i][2];


	cout << " level " << level << " ind "<< ind << "simmax " << simmax <<  endl;


	/*if (im2>0)
	{*/
	int k =0;
	int flag = 0;
	vpImagePoint pI0;
while(im[k]>0)
{
	char buf0[FILENAME_MAX];
	int nim = level*1000+im[k];
			sprintf(buf0, opath.c_str(),nim);
			std::string filename0(buf0);
			vpImageIo::readPPM(IT0, filename0);

			/*cout << " pIi " << pI.get_i() << " pIj " << pI.get_j() << endl;
			getchar();*/


			for (int cc=0; cc<5;cc++)
			{
				for (int rr=0; rr<5;rr++)
				{
				pI0.set_i(pI.get_i()-(int)(nr/2 - rr*intr-intr/2)/2);
				pI0.set_j(pI.get_j()-(int)(nc/2 - cc*intc-intc/2)/2);
				for(int rot=0;rot<10;rot++)
				{
					ang=Ang-(rotm-rot*(rotm/5));
					rotateTemp(IT0,ang);
				//pointsPos.push_back(pI);
				sim0 = computeSimilarityPos(argImDT,&IT0,pI0);
				cout << " simmo " << sim0 << " pI0i " << pI0.get_i() << " pI0j " << pI0.get_j() << "rot "<<ang<< endl;
				if (sim0<simmaxP)
				{
				simmaxP = sim0;
				Popt0 = pI0;
				AngOpt0 = ang;
				}
				}
			}
			}
			sim0=simmaxP;

			count++;
			//std::cout << " simmax " << simmax << std::endl;
			if(sim0<simmax)
			{
				kmax = k;
				simmax = sim0;
				Popt=Popt0;
				AngOpt = AngOpt0;
				flag = 1;
			}
			//std::cout << "sim0 " << sim0 << std::endl;
			//cout << " pIi " << Popt0.get_i() << " pIj " << Popt0.get_j() << endl;
k++;
}
int imax = im[kmax];
//std::cout << "imax: " << imax << std::endl;
im.resize(20,true);
arb = 0;
if (flag == 1)
{
while (hierarchy[i-npl+imax][3+arb]>0)
{
	//std::cout << "ind: " <<i-npl+imax << " flag " << flag << std::endl;
im[0]=hierarchy[i-npl+imax][1];
im[arb+1]= hierarchy[i-npl+imax][3+arb];
arb++;
}
}
else {
	while (hierarchy[i-npl+ind][3+arb]>0)
	{
	im[0]=hierarchy[i-npl+ind][1];
	im[arb+1]= hierarchy[i-npl+ind][3+arb];
	arb++;
	}
}

			//cout << " ok " << im << endl;

	        m=0;
	        simmax=2000;
	        simmaxP=2000;
	        nr=(int)nr/2;
	        intr = (int)intr/2;
	        nc = (int)nc/2;
	        intc = (int)intc/2;
	        rotm=rotm/2;

			while (hierarchy[i-m][3]==0)
			{
	           //int indi=hierarchy[i-m][1];

				char buf[FILENAME_MAX];
				//sprintf(buf, opath.c_str(),level*1000+indi);
				sprintf(buf, opath.c_str(),level*1000+npl-m);
				std::string filename(buf);
							//std::cout << "Write: " << filename << std::endl;
				vpImageIo::readPPM(IT0, filename);

				for (int cc=0; cc<5;cc++)
				{
					for (int rr=0; rr<5;rr++)
					{
						pI0.set_i(pI.get_i()-(int)(nr/2 - rr*intr-intr/2)/2);
						pI0.set_j(pI.get_j()-(int)(nc/2 - cc*intc-intc/2)/2);
						for(int rot=0;rot<10;rot++)
										{
											ang=Ang-(rotm-rot*(rotm/5));
											rotateTemp(IT0,ang);
					//pointsPos.push_back(pI);
					sim = computeSimilarityPos(argImDT,&IT0,pI0);
					if (sim<simmaxP)
					{
					simmaxP = sim;
					Popt0 = pI0;
					AngOpt0=AngOpt;
					}
										}
				}
				}
				sim=simmaxP;

				count++;
				cout << "sim " << sim << " n " << level*1000+npl-m << endl;
				if (sim<simmax)
					{
						simmax=sim;
						ind=hierarchy[i-m][1];
						Popt=Popt0;
						AngOpt=AngOpt0;
					}

				//underT.push_back(hierarchy[i-m][1]);
				m++;
			}
			pI=Popt;
			Ang=AngOpt;

			cout << " im " << im << " simmax "<< simmax  << "popti " << Popt.get_i() << "poptj "<<Popt.get_j() << " AngOpt " << AngOpt << endl;
	i-=npl;
	}
}


void apViewGeneration::detectMS1(vpImage<unsigned char> &I0, std::string opath)
	{
	//std::vector< const vpImage<vpRGBa>* >& Ipyramid;
	//Ipyramid.resize(0);
	edgeOrientMap(I0);
	vpImage<unsigned char> I1(480,640);
	vpImageIo::writePPM(I0, "inputedge.pgm");
	vpImage<vpRGBa> *argImDT;
	vpImage<vpRGBa> *argImDTP;
	argImDT=dt0(&I0);
	initPyramid(argImDT,Ipyramid);
	std::vector<vpImagePoint> pointsPos;
	argImDTP = Ipyramid[3];
	std::cout << " size " << argImDTP->getHeight() << std::endl;
	const char *filename = "hierarchy3.txt";
		std::fstream file;
		I1.resize(argImDTP->getHeight(),argImDTP->getWidth());
		  for (int x = 0; x < argImDTP->getWidth(); x++) {
		    for (int y = 0; y < argImDTP->getHeight(); y++) {
		      I1[y][x] = (*argImDTP)[y][x].B;
		    }
		  }
    vpImageIo::writePPM(I1, "DTI.pgm");
    /*I0.resize(argImDTP->getHeight(),argImDTP->getWidth());
	  for (int x = 0; argImDTP->getWidth(); x++) {
	    for (int y = 0; y < argImDTP->getHeight(); y++) {
	    	if(I0[y][x]!=100)
	    	{
	      I1[y][x] = 0;
	    	}
	    	else
	    	{
	  	      I1[y][x] = 255;
	    	}
	    }
	  }
vpImageIo::writePPM(I1, "EdgeM.pgm");*/
	vpImage<unsigned char> IT0;
	vpImage<unsigned char> IT1;
	int level;
	int im1;
	int im2;
	int pos;
	int npl;
	int kmax;
	vpColVector im(20);
	string ap;
	vpMatrix hierarchy;
	hierarchy.loadMatrix(filename,hierarchy,false);
	cout << hierarchy << endl;

	int size;
	int count=0;
	double sim,sim0,sim1;
	double simmax=100;
	vpImage<unsigned char> IT;

	int i=hierarchy.getRows()-1;

		level = hierarchy[i][0]+1;
		unsigned int cScale = static_cast<unsigned int>(pow(2., (int)(level)));
		vpImagePoint pI;
		int nc = (int)640/cScale;
		int nr = (int)480/cScale;
		int intc = (int)nc/4;
		int intr = (int)nr/4;
		pointsPos.resize(0);

		cout << " level " << level << endl;
		int j=1;
		int ind=0;
		int m;
		int simmaxP=200;
		double imoy = 0;
		double jmoy = 0;
		std::vector<int> underT;
		vpImagePoint Popt;
        vpImagePoint Popt0;
		if (level>1)
		argImDTP = Ipyramid[level+1-1];
		else
		argImDTP = Ipyramid[1];
		while(hierarchy[i-j+1][0]==level-1){
		//while(j<64){
			char buf[FILENAME_MAX];
			sprintf(buf, opath.c_str(),level*1000+j);
			//sprintf(buf, opath.c_str(),j-1);
			std::string filename(buf);
			//std::cout << "Write: " << filename << std::endl;
			vpImageIo::readPPM(IT, filename);
			cout << " level0 " << argImDTP->getHeight() << " level1 " << IT.getHeight() << endl;
			for (int cc; cc<4;cc++)
			{
				for (int rr; rr<4;rr++)
				{
				pI.set_i(rr*intr+(int)intr/2);
				pI.set_j(cc*intc+(int)intc/2);
				//pointsPos.push_back(pI);
				sim = computeSimilarityPos(argImDTP,&IT,pI);
				if (sim<simmaxP)
				{
				simmaxP = sim;
				Popt = pI;
				}
				}
			}
				sim=simmaxP;

			count++;
			cout << " simi "<< sim << " " << j << endl;
	if(sim<simmax)
	{
		simmax=sim;
		ind=j;
	}
	j++;
		}
		//getchar();
		cout << " okd "<< ind << " " << i << " " << j << endl;
		int arb =0;
		if(hierarchy[i-j+ind+1][3]>0)
		{
	 while (hierarchy[i-j+ind+1][3+arb]>0)
	 {
	im[0]=hierarchy[i-j+ind+1][1];
		 //im[0]=8;
	im[arb+1]= hierarchy[i-j+ind+1][3+arb];
	arb++;
	 }
		}
		else {
			im[0]=hierarchy[i-j+ind+1][1];
		}
	/*if (im2==0)
	{
		im2=im1;
	}*/
	i-=j-1;
	cout << " okd " << im[0] << " okd1 " << im[1] << endl;
	simmax=2000;
	simmaxP=2000;
	ind=0;

		while (i>-1)
		{

	level = hierarchy[i][0]+1;

	if (level>1)
	{argImDTP = Ipyramid[level+1-1];}
	else
	{argImDTP = Ipyramid[1];}

	npl = hierarchy[i][2];


	cout << " level " << level << " ind "<< ind << "simmax " << simmax <<  endl;


	/*if (im2>0)
	{*/
	int k =0;
	int flag = 0;
	vpImagePoint pI0;
while(im[k]>0)
{
	char buf0[FILENAME_MAX];
	int nim = level*1000+im[k];
			sprintf(buf0, opath.c_str(),nim);
			std::string filename0(buf0);
			vpImageIo::readPPM(IT0, filename0);

			for (int cc; cc<4;cc++)
			{
				for (int rr; rr<4;rr++)
				{
				pI0.set_i(pI.get_i()*2-(int)nr/2 + rr*intr+(int)intr/2);
				pI0.set_j(pI.get_j()*2-(int)nc/2 + cc*intc+(int)intc/2);
				//pointsPos.push_back(pI);
				sim0 = computeSimilarityPos(argImDTP,&IT0,pI0);
				if (sim0<simmaxP)
				{
				simmaxP = sim0;
				Popt0 = pI0;
				}
			}
			}
			sim0=simmaxP;

			count++;
			std::cout << "simmax " << simmax << std::endl;
			if(sim0<simmax)
			{
				kmax = k;
				simmax = sim0;
				Popt=Popt0;
				flag = 1;
			}
			std::cout << "sim0 " << sim0 << std::endl;
k++;
}
int imax = im[kmax];
std::cout << "imax: " << imax << std::endl;
im.resize(20,true);
arb = 0;
if (flag == 1)
{
while (hierarchy[i-npl+imax][3+arb]>0)
{
	//std::cout << "ind: " <<i-npl+imax << " flag " << flag << std::endl;
im[0]=hierarchy[i-npl+imax][1];
im[arb+1]= hierarchy[i-npl+imax][3+arb];
arb++;
}
}
else {
	while (hierarchy[i-npl+ind][3+arb]>0)
	{
	im[0]=hierarchy[i-npl+ind][1];
	im[arb+1]= hierarchy[i-npl+ind][3+arb];
	arb++;
	}
}

			cout << " ok " << im << endl;

	        m=0;
	        simmax=2000;
	        simmaxP=2000;

			while (hierarchy[i-m][3]==0)
			{
	           //int indi=hierarchy[i-m][1];

				char buf[FILENAME_MAX];
				//sprintf(buf, opath.c_str(),level*1000+indi);
				sprintf(buf, opath.c_str(),level*1000+npl-m);
				std::string filename(buf);
							//std::cout << "Write: " << filename << std::endl;
				vpImageIo::readPPM(IT0, filename);

				for (int cc; cc<4;cc++)
				{
					for (int rr; rr<4;rr++)
					{
					pI0.set_i(pI.get_i()*2-(int)nr/2 + rr*intr+(int)intr/2);
					pI0.set_j(pI.get_j()*2-(int)nc/2 + cc*intc+(int)intc/2);
					//pointsPos.push_back(pI);
					sim = computeSimilarityPos(argImDTP,&IT0,pI0);
					if (sim<simmaxP)
					{
					simmaxP = sim;
					Popt0 = pI0;
					}
				}
				}
				sim=simmaxP;

				count++;
				cout << "sim " << sim <<  endl;
				if (sim<simmax)
					{
						simmax=sim;
						ind=hierarchy[i-m][1];
						Popt=Popt0;
					}


				//underT.push_back(hierarchy[i-m][1]);
				m++;
			}
			pI=Popt;

			cout << " im " << im << " simmax "<< simmax  << "popti " << Popt.get_i() << "poptj "<<Popt.get_j() << endl;
	i-=npl;
	}
}

double apViewGeneration::computeSimilarityPos(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT, vpImagePoint pI)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int m,mo,yy,xx,adtIy,adtIx;
		double cor,t0,t1,dist,delta,oriT,oriI,dtI;
		int offs =(int) 127*((double)width/640);
		//int offs =(int) 127;
		t0= vpTime::measureTimeMs();
		dist=0;
		mo=0;
		m=0;
		for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {

			  if (((*IT)[y][x]!=100) && y+(int)(pI.get_i()-height/2)>0 && y+(int)(pI.get_i()-height/2)<480 &&  x+(int)(pI.get_j()-width/2)>0 && x+(pI.get_j()-width/2)<640)
		    {
				  if(y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2 >0 && y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2 <480 && x-(int)(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2 > 0 && x-(int)(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2 <640)
				  {
				  /*cout << " h " <<pI.get_i()-height/2 << endl;
				  cout << " w " <<pI.get_j()-width/2 << endl;
				  getchar();*/
				  //if ((double)(*Im)[y][x].B<255)

		    	m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];
		    	oriI = (double)(*Im)[y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2][x-(int)(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2].A;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{
		    		oriI=(double)(*Im)[y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2+yy][x-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2 + xx].A;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2+yy][x-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2 + xx].A;
						xx ++;
					}
		    		}
		    	(*Im)[y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2][x-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2].A = oriI;
		    	if (oriI>100 || oriI<100)
		    		{
		    		mo++;
		    		delta=abs((*IT)[y][x]-oriI);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		if (delta>255-delta) delta=255-delta;
			   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
		    		//cout  <<" " << delta << endl;
		    	//dist = dist + dtI + 0.1*delta;
		    		dist = dist + (double)(*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)].B + 0.3*delta;
		    		//cout << mo << endl;
		    	//dist = dist  + 0.1*delta;
		    		}
		    	/*else
		    	dist = dist + (ImDT)[y][x];*/
		    }

		    }
		  }
		}

		dist=dist/mo;
		t1= vpTime::measureTimeMs();
		//cout << " timem " << t1-t0 << endl;

		return dist;
}







/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
