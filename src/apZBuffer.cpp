#include "apZBuffer.h"


void
apZBuffer::init()
{

  rowNum = 0  ;
  colNum = 0 ;

  data = NULL ;
  rowPtrs = NULL ;

  dsize = 0 ;
  trsize =0 ;
  znear=0;
  zfar=0;
  borni1=480;
  borni2=0;
  bornj1=640;
  bornj2=0;
}



apZBuffer::apZBuffer() : vpMatrix()
{
init();
}


/*apZBuffer::apZBuffer(const vpImage<unsigned char> &I) : vpMatrix()
{
init(I);
}*/

apZBuffer::apZBuffer(const apZBuffer &B) : vpMatrix()
{
init() ;
*this = B ;
}

void
apZBuffer::set(const int r,const int c,const double zn, const double zf)
{
int i,j;
resize(r,c);
znear=zn;
zfar=zf;
for (i=0 ; i < r ; i++)
for (j=0 ; j < c ; j++)
(*this)[i][j]=1;
}


void
apZBuffer::getDepth()
{
int h = rowNum;
int w = colNum;
float pixels[h][w];
double val;
glReadPixels(0,0,w,h,GL_DEPTH_COMPONENT,GL_FLOAT,&pixels);
int i,j;
for (i=0 ; i < h ; i++){
for (j=0 ; j < w ; j++){
val=pixels[480-i-1][j];
(*this)[i][j]=val;
}
}
}


void
apZBuffer::getDepth(vpImage<double> &I,vpImage<unsigned char> &Ic)
{
int h = rowNum;
int w = colNum;
float pixels[h][w];
double val;
glReadPixels(0,0,w,h,GL_DEPTH_COMPONENT,GL_FLOAT,&pixels);
int i,j;
for (i=0 ; i < h ; i++){
for (j=0 ; j < w ; j++){
val=pixels[480-i-1][j];
(*this)[i][j]=val;
I[i][j]=val;
Ic[i][j]=(unsigned char)(floor(val*255));
}
}
}

void
apZBuffer::getLuminance(vpImage<double> &I,vpImage<unsigned char> &Ic)
{
int h = rowNum;
int w = colNum;
float pixels[h][w];
double val;
glReadPixels(0,0,w,h,GL_LUMINANCE,GL_FLOAT,&pixels);
int i,j;
for (i=10 ; i < h ; i++){
for (j=20 ; j < w ; j++){
val=pixels[480-i-1-10][j-20];
//(*this)[i][j]=val;
//I[i][j]=val;
Ic[i][j]=(unsigned char)floor((val*255));
}
}
}


void
apZBuffer::getCastLuminance(vpImage<unsigned char> &Ic)
{
int h = rowNum;
int w = colNum;

float pixels[h][w];
double val;
glReadPixels(0,0,w,h,GL_LUMINANCE,GL_FLOAT,&pixels);
int i,j;
for (i=0 ; i < borni2-borni1+60 ; i++){
for (j=0 ; j < bornj2-bornj1+60 ; j++){
val=pixels[480-i-1-borni1+30][j+bornj1-30];
Ic[i][j]=(unsigned char)floor((val*255));
}
}
}

void
apZBuffer::getCastRGBLuminance(vpImage<unsigned char> &Ic,vpImage<vpRGBa> &Ip,vpMatrix &Zcoord,const double dist)
{
znear=dist-0.06;
zfar=dist+0.06;
int h = rowNum;
int w = colNum;
borni1=30;
borni2=450;
bornj1=30;
bornj2=610;
//float pixels[borni2-borni1+60][bornj2-bornj1+60][3];
float pixels[h][w][3];
float pixels1[h][w];
double r,g,b,lum,Z;
glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT,&pixels);
glReadPixels(0,0,w,h,GL_DEPTH_COMPONENT,GL_FLOAT,&pixels1);
//glReadPixels(bornj1-30,480-1-borni2+30,bornj2-bornj1+60,borni2-borni1+60,GL_RGB,GL_FLOAT,&pixels);
int i,j;

	for (i=0 ; i < borni2-borni1+60 ; i++){
		for (j=0 ; j < bornj2-bornj1+60 ; j++){
			r=pixels[480-i-1-borni1+30][j+bornj1-30][0];
			g=pixels[480-i-1-borni1+30][j+bornj1-30][1];
			b=pixels[480-i-1-borni1+30][j+bornj1-30][2];
			/*r=pixels[borni2-borni1+60-i-1][j][0];
			g=pixels[borni2-borni1+60-i-1][j][1];
			b=pixels[borni2-borni1+60-i-1][j][2];*/
			lum=r+0.66*g+0.33*b;
			//cout<< r << " " << g<< " " << b << endl;
			Ic[i][j]=(unsigned char)floor((lum*255));
			Ip[i][j].R=(unsigned char)floor((r*255));
			Ip[i][j].G=(unsigned char)floor((g*255));
			Ip[i][j].B=(unsigned char)floor((b*255));
			if(pixels1[480-i-1-borni1+30][j+bornj1-30]<1)
			{//Zcoord[i][j] = (2*znear*zfar)/((1-(*this)[i][j])*(zfar+znear));
				Zcoord[i][j] = -(znear*zfar)/(pixels1[480-i-1-borni1+30][j+bornj1-30]*(zfar-znear)-zfar);
			}
			else {Zcoord[i][j]=zfar;}

			}
	}
}


void
apZBuffer::getRGBDepth(vpImage<double> &Ic,vpImage<unsigned char> &Ic1, vpImage<unsigned char> &Ic2,vpImage<vpRGBa> &Ip,vpMatrix &Zcoord,const double dist, const double &m)
{
//znear=dist-0.04;
//zfar=dist+0.04;
/*znear=dist-30;
zfar=dist+30;*/
/*znear=dist-0.20;
zfar=dist+0.20;
*/
znear=dist-m;
zfar=dist+m;


/*znear=dist-1;
zfar=dist+1;*/

int h = rowNum;
int w = colNum;
//float pixels[borni2-borni1+60][bornj2-bornj1+60][3];
float pixels[h][w][3];
float pixels1[h][w];
double r,g,b,lum,Z;
double t0= vpTime::measureTimeMs();
glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT,&pixels);
glReadPixels(0,0,w,h,GL_DEPTH_COMPONENT,GL_FLOAT,&pixels1);
int i,j;
	for (i=0 ; i < h ; i++){
		for (j=0 ; j < w ; j++){
			r=pixels[480-i-1][j][0];
			g=pixels[480-i-1][j][1];
			b=pixels[480-i-1][j][2];
			//lum=r+0.66*g+0.33*b;
			//Ic[i][j]=(unsigned char)floor((lum*255));
			Ip[i][j].R=(unsigned char)floor((r*255));
			Ip[i][j].G=(unsigned char)floor((g*255));
			Ip[i][j].B=(unsigned char)floor((b*255));
			Ic1[i][j]=(unsigned char)255*pixels1[480-i-1][j];
			Ic2[i][j]=(unsigned char)(0.5*floor((r*255))+0.3*floor((g*255))+0.1*floor((b*255)));
			Ic[i][j]=pixels1[480-i-1][j];
			if(pixels1[480-i-1][j]<1)
			{
				Zcoord[i][j] = -(znear*zfar)/(pixels1[480-i-1][j]*(zfar-znear)-zfar);
			}
			else {Zcoord[i][j]=zfar;}
			}
	}
	double t1= vpTime::measureTimeMs();
	cout << " render "<<t1-t0<<endl;
}


void
apZBuffer::cast(vpImage<unsigned char> &Id0,vpImage<unsigned char> &Id,vpImage<unsigned char> &Ic)
{
int h = rowNum;
int w = colNum;
borni1=480;
borni2=0;
bornj1=640;
bornj2=0;
float pixels[h][w];
double val;
glReadPixels(0,0,w,h,GL_LUMINANCE,GL_FLOAT,&pixels);
int i,j;
for (i=0 ; i < h ; i++){
for (j=0 ; j < w ; j++){
val=pixels[480-i-1][j];
if (val>0)
{
	if (i<borni1)
	{
		if (i>29)
	{borni1=i;}
		else {borni1=30;}
	}
	if (i>borni2)
	{
		if (i<451)
	{borni2=i;}
		else {borni2=450;}
	}
	if (j<bornj1)
	{
		//if (j>109)
		if (j>29)
	{bornj1=j;}
		//else {bornj1=111;}
		else {bornj1=30;}
	}
	if (j>bornj2)
	{
		if (j<611)
	{bornj2=j;}
		else {bornj2=610;}
	}
}
}
}
Ic.resize(borni2-borni1+60,bornj2-bornj1+60);
Id0.resize(borni2-borni1+60,bornj2-bornj1+60);
for (i=0 ; i < borni2-borni1+60 ; i++){
for (j=0 ; j < bornj2-bornj1+60 ; j++){
Id0[i][j]=Id[i+borni1-30][j+bornj1-30];
}
}

}




void
apZBuffer::getDepth(vpImage<unsigned char> &Ic)
{
int h = rowNum;
int w = colNum;
float pixels[h][w];
double val;
glReadPixels(0,0,w,h,GL_DEPTH_COMPONENT,GL_FLOAT,&pixels);
int i,j;
for (i=0 ; i < h ; i++){
for (j=0 ; j < w ; j++){
val=pixels[480-i-1][j];
(*this)[i][j]=val;
Ic[i][j]=(unsigned char)(val*255);
}
}
}

void
apZBuffer::dImage(vpImage<double> &I)
{
double val;
int i,j;
for (i=0 ; i < rowNum ; i++){
for (j=0 ; j < colNum ; j++){
val = (*this)[i][j];
I[i][j]=val;
}
}
}
void 
apZBuffer::displayZB(vpImage<unsigned char> &I)
{
double val;
double val2;
int i,j;
for (i=0 ; i < rowNum ; i++){
for (j=0 ; j < colNum ; j++){
val = (*this)[i][j];
val2 = floor(val*255);
I[i][j]=(unsigned char)val2;

}
}
vpDisplay::display(I);
//vpDisplay::flush(I);
}

vpMatrix
apZBuffer::getZ(const double dist)
{
znear=dist-0.04;
	//znear=dist-0.40;
std::cout<<"dist = "<<dist<<std::endl;
zfar=dist+0.04;
//zfar=dist+0.4;
ZCoord.resize(rowNum,colNum);
int i,j;
for (i=0 ; i < rowNum ; i++){
for (j=0 ; j < colNum ; j++){
	if((*this)[i][j]<1)
	{//ZCoord[i][j] = (2*znear*zfar)/((1-(*this)[i][j])*(zfar+znear));
		ZCoord[i][j] = -(2*znear*zfar)/((*this)[i][j]*(zfar-znear)-zfar);
	}
	else {ZCoord[i][j]=zfar;}
}
}
return ZCoord;
}





