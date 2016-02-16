#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPoint.h>
#include <visp/vpFeatureDisplay.h>


#include "apDomOrientation.h"
#include "apImageFilter.h"


using namespace std ;





/*!
  Initialize the memory space requested for vpFeatureLuminance visual feature.
*/
void
apDomOrientation::init()
{
Orientations.resize(1);
OrientationsTemp.resize(1);
}


void
apDomOrientation::init(int r_size, int k, int nb, double th)
{
init() ;
region_size=r_size;
nbor=k;
threshold=th;
nbar=nb;
nb_region = 0;
nb_regioni = 0;
nb_regionj = 0;
nb_regionT = 0;
nb_regionTi = 0;
nb_regionTj = 0;
nb_regionI = 0;
nb_regionIi = 0;
nb_regionIj = 0;
}

/*! 
  Default constructor that build a visual feature.
*/
apDomOrientation::apDomOrientation() 
{
    init();
}

/*! 
  Default destructor.
*/
apDomOrientation::~apDomOrientation()
{
}

void
apDomOrientation::buildFrom(vpImage<unsigned char> &I, vpImage<unsigned char> &Iout, unsigned int u)
{

imIx.resize(I.getHeight(),I.getWidth());
imIy.resize(I.getHeight(),I.getWidth());
Iout.resize(I.getHeight(),I.getWidth());
Iout.resize(I.getHeight(),I.getWidth());
Ori.resize(I.getHeight(),I.getWidth());
vpColVector DomOri(nbar);
for (int l=0;l<nbar;l++)
{
	DomOri[l]=l;
}
vpColVector Orient(nbar);
if (u==0){
nb_regionIi=(int)I.getHeight()/region_size;
nb_regionIj = (int)I.getWidth()/region_size;
nb_regionI=nb_regionIi*nb_regionIj;
}
else
{
nb_regionTi = (int)I.getHeight()/region_size;
nb_regionTj = (int)I.getWidth()/region_size;
nb_regionT = nb_regionTi*nb_regionTj;
}
nb_regioni=(int)I.getHeight()/region_size;
nb_regionj = (int)I.getWidth()/region_size;
nb_region=nb_regioni*nb_regionj;
//OriMat.resize(nb_regioni,nb_regionj);
//OriMatTemp.resize(nb_regioni,nb_regionj);
cout << "nb regions "<< nb_regionT << endl;
cout << "region_size "<< region_size << endl;
double Ix,Iy;
      for (int i=3; i < I.getHeight()-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < I.getWidth()-3; j++)
	    { 
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix = 1 * apImageFilter::sobelFilterX(I,i,j);
	      Iy = 1 * apImageFilter::sobelFilterY(I,i,j);
	      imIx[i][j] = Ix;
	      imIy[i][j] = Iy;
	      if (sqrt(vpMath::sqr(Ix)+vpMath::sqr(Iy))>threshold)
	      {
	      Ori[i][j]=atan(Iy/Ix);}
              else 
              {Ori[i][j] = 10;}
              
	    }
	}

	for (int k=0;k<nb_region;k++)
	{
    Orient.resize(nbar);
		for (int n=0;n<region_size;n++)
		{
			for (int m=0;m<region_size;m++)
			{
			//normgrad=sqrt(vpMath::sqr(imIx[n+region_size*(floor(k/nb_regioni))][m+regionsize*(k-nb_regioni*floor(k/nb_regioni))])+vpMath::sqr(imIy[n+region_size*(floor(k/nb_regioni))][m+regionsize*(k-nb_regioni*floor(k/nb_regioni))]));
				//if (normgrad>normax[0])
				int l =0;
				int nR=((int)floor(k/nb_regionj));
				int mR=(k-nb_regioni*(int)floor(k/nb_regioni));
				int nI=n+region_size*nR;
				int mI=m+region_size*mR;
				//cout<<" nI "<<nI<<"mI"<<mI<<endl;
				//cout << " ori " << Ori[nI][mI] << " gradmag " << sqrt(vpMath::sqr(imIx[nI][mI])+vpMath::sqr(imIy[nI][mI]))<< endl;
				while(Ori[nI][mI]>-M_PI/2+l*(M_PI/nbar))
				{
					l++;
				}
				if(Ori[nI][mI]<10 && nI>3 && nI<I.getHeight()-4 && mI>3 && mI<I.getWidth()-4 )
				//if(!Ori[n+region_size*((int)floor(k/nb_regioni))][m+region_size*(k-nb_regioni*(int)floor(k/nb_regioni))]==1)
				{Orient[l]=Orient[l]+1;
				//cout<<" l "<<l<<" n "<< nI <<" m "<<mI<< " ori "<< Ori[nI][mI] <<" gradx "<< imIx[nI][mI] << " grady " << imIy[nI][mI] << endl;
				}

			}
		}
		//cout<<"orient "<< Orient <<endl;

   int nb_permutation = 1 ;
   int i = 0 ;
   while (nb_permutation !=0)
   {
     nb_permutation = 0 ;
     for (int j =Orient.getRows()-1 ; j >= i+1 ; j--)
     {
       if ((Orient[j]>Orient[j-1]))
       {
     double tmp = Orient[j] ;
     Orient[j] = Orient[j-1] ;
     Orient[j-1] = tmp;
     tmp=DomOri[j];
     DomOri[j]=DomOri[j-1];
     DomOri[j-1]=tmp;
     nb_permutation++ ;
       }
     }
     i++ ;
   }

   for (int n=0;n<region_size;n++)
   		{
   			for (int m=0;m<region_size;m++)
   			{
   				int l0 =0;
   				int nR0=((int)floor(k/nb_regionj));
   				int mR0=(k-nb_regioni*(int)floor(k/nb_regioni));
   				int nI0=n+region_size*nR0;
   				int mI0=m+region_size*mR0;
   				Iout[nI0][mI0]=((Orient[0]+M_PI/2)*(255/M_PI));
   			}
   		}

   //DomOri.resize(nbor, false);
   //cout<<"Dom Ori "<<DomOri<<endl;
   if (u==0){
   Orientations.push_back(DomOri);
   }
	   //OriMat.add(DomOri,nR,mR);}
   if (u==1)
   {
    OrientationsTemp.push_back(DomOri);
    //std::cout<<" ok " <<std::endl;
   }
	   //OriMatTemp.add(DomOri,nR,mR);}

   //getchar();
}


}

vpColVector apDomOrientation::computeCostFunction()

{

std::cout<<nb_regionI<<std::endl;
std::cout<<nb_regionT<<std::endl;
int max =0;
vpColVector Fmax(2);
for (int k=1;k<nb_regionI;k++)
{
int Fcout=0;
	for (int l=1;l<nb_regionT;l++)
	{
		if (k-nb_regionIi*floor(k/nb_regionIi)<nb_regionIi-nb_regionTi && floor(k/nb_regionIi)<nb_regionIj-nb_regionTj)
		{
			vpColVector DoOriTemp = OrientationsTemp[4];
			vpColVector DoOri=Orientations[k+nb_regionIi*floor(l/nb_regionTi)+l-nb_regionTi*floor(l/nb_regionTi)];
			//std::cout<<Orientations.size()<<std::endl;
			//std::cout<<OrientationsTemp.size()<<std::endl;
			//std::cout<<DoOriTemp[0]<<std::endl;
			//std::cout<<DoOri[1]<<std::endl;
			//getchar();
			for (int m=0;m<nbor;m++)
			{
				if (DoOriTemp[m]==DoOri[0])
				{
					Fcout++;
				}
			}
		}
	}

	if (Fcout > max)
		{
		Fmax[0]=Fcout;
		Fmax[1]=k;
		max=Fcout;
		}

}
std::cout<<"ok"<<std::endl;
return Fmax;
}





/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
