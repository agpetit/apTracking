#include "apEdgeMap.h"

void
apEdgeMap::edgeMapLap(const vpImage<double> &I, vpImage<unsigned char> &If, const double th)
{
vpImage<double> Iu;
vpImage<double> Iv;
int h=I.getHeight();
int w=I.getWidth();
Iu.resize(I.getHeight(),I.getWidth());
Iv.resize(I.getHeight(),I.getWidth());
vpMatrix M(3,3);
	M=0;
	M[0][1]=1;
	M[1][0]=1;
	M[1][2]=1;
	M[1][1]=-4;
	M[2][1]=1;
apImageFilter::filter(I,Iu,Iv,M);
int n,m;
for (n=0;n<h;n++){
				for (m=0;m<w;m++){
if (Iu[n][m]*255>th) {If[n][m]=255;}//{If[n][m]=255;}
else {If[n][m]=0;}
}
}
}

void
apEdgeMap::edgeMapLap(const vpImage<double> &I, const vpImage<unsigned char> &I1, vpMatrix &Ori, vpImage<unsigned char> &If, const double th)
{
vpImage<double> Iu;
vpImage<double> Iv;
vpImage<double> imG(480,640);
double a,b;
int h=I.getHeight();
int w=I.getWidth();
Iu.resize(I.getHeight(),I.getWidth());
Iv.resize(I.getHeight(),I.getWidth());
vpMatrix M(3,3);
	M=0;
	M[0][1]=1;
	M[1][0]=1;
	M[1][2]=1;
	M[1][1]=-4;
	M[2][1]=1;
apImageFilter::filter(I,Iu,Iv,M);
int n,m;

/*for (n=3; n < h-3 ; n++)
{
for (m = 3 ; m < w-3; m++)
  {
    imG[n][m] =   apImageFilter::gaussianFilter(I1,n,m);
  }
}*/


for (int i=3;i<h-3;i++)
		{
			for (int j=3;j<w-3;j++)
			{
				a=(apImageFilter::sobelFilterX(I1,i,j));
				b=(apImageFilter::sobelFilterY(I1,i,j));
				//if (sqrt((vpMath::sqr(a) + vpMath::sqr(b)))>1)
			    if ( (!a==0 || !b==0) && sqrt((vpMath::sqr(a) + vpMath::sqr(b)))>0) //
				{
			    	/*if(atan(b/a)<0)*/
			    	{
				 Ori[i][j]=(atan(b/a));
			    	}
			    	/*else
			    					    	{
			    						 Ori[i][j]=(atan(b/a));
			    					    	}*/

				 //std::cout<<" a "<<Ori[i][j]*180/M_PI <<std::endl;
				}
				 else {Ori[i][j]=0;}
				if (Iu[i][j]*255>th) {If[i][j]=255;}
			    //if (Iu[i][j]*255>3) {If[i][j]=255;}
				else {If[i][j]=0;}
			}
		}

}


void
apEdgeMap::edgeMapLap2(const vpImage<unsigned char> &I1, vpMatrix &Ori, vpImage<unsigned char> &If, const double th)
{

vpImage<double> imG(480,640);
double c,d,e;
int h=I1.getHeight();
int w=I1.getWidth();
vpMatrix M(3,3);
	M=0;
	M[0][1]=1;
	M[1][0]=1;
	M[1][2]=1;
	M[1][1]=-4;
	M[2][1]=1;
unsigned int size = M.getRows() ;
unsigned int half_size = floor(size/2) ;
//apImageFilter::filter(I,Iu,Iv,M);
int n,m;
/*for (n=3; n < h-3 ; n++)
{
for (m = 3 ; m < w-3; m++)
  {
    imG[n][m] =   apImageFilter::gaussianFilter(I1,n,m);
  }
}*/


for (int i=3;i<h-3;i++)
		{
			for (int j=3;j<w-3;j++)
			{
			      /*double   conv_u = 0 ;
			      for(unsigned int a = 0 ; a < size ; a++)
			        for(unsigned int b = 0 ; b < size ; b++)
				{
				  double val =  (double)I1[i-half_size+a][j-half_size+b]/255;
				  conv_u += M[a][b]*val;
				}*/
					c=(apImageFilter::sobelFilterX(I1,i,j));
					d=(apImageFilter::sobelFilterY(I1,i,j));
					e=apImageFilter::lapFilter(I1,i,j);
				//if (sqrt((vpMath::sqr(a) + vpMath::sqr(b)))>1)
			    if ( (!c==0 || !d==0) && sqrt((vpMath::sqr(c) + vpMath::sqr(d)))>0) //
				{
			    	/*if(atan(b/a)<0)*/
			    	{
				 Ori[i][j]=(atan(d/c));
			    	}
				}
				 else {Ori[i][j]=0;}
				if (e>th) {If[i][j]=255;}
				else {If[i][j]=0;}

			}
		}

}


void
apEdgeMap::edgeMapLap(const vpImage<unsigned char> &I, vpImage<unsigned char> &If, const double th)
{
int h=I.getHeight();
int w=I.getWidth();
vpMatrix M(3,3);
	M=0;
	M[0][1]=1;
	M[1][0]=1;
	M[1][2]=1;
	M[1][1]=-4;
	M[2][1]=1;
apImageFilter::filterTh(I,If,M,th);
}

void apEdgeMap::edgeOrientMap(vpImage<unsigned char> &I0)
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






