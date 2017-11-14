

#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImageFilter.h>

#include "vpFeatureGradient.h"


using namespace std ;


/*!
  \file vpFeatureGradient.cpp
  \brief Class that defines the image luminance visual feature

  for more details see
  C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/



/*!
  Initialize the memory space requested for vpFeatureGradient visual feature.
*/
void
vpFeatureGradient::init()
{
    nbParameters = 1;
    dim_s = 0 ;
    bord = 10 ;

    if (flags == NULL)
      flags = new bool[nbParameters];
    for (int i = 0; i < nbParameters; i++) flags[i] = false;

    //default value Z (1 meters)
    Z = 1;

    firstTimeIn =0 ;

    dim_s0 = 0 ;
    dim_s1 = 0 ;

}


void
vpFeatureGradient::init(int _nbr, int _nbc, double _Z)
{
  init() ;

  nbr = _nbr ;
  nbc = _nbc ;
  // number of feature = nb column x nb lines in the images
  dim_s = (nbr-2*bord)*(nbc-2*bord) ;
  imIx.resize(nbr,nbc) ;
  imIy.resize(nbr,nbc) ;
  imLap.resize(nbr,nbc) ;
  imG.resize(nbr,nbc) ;

  s.resize(dim_s) ;
  
  pixInfo = new vpGradient[dim_s] ;

  dim_s0 = (nbr-2*bord)*(nbc-2*bord) ;
  imIx0.resize(nbr,nbc) ;
  imIy0.resize(nbr,nbc) ;
  imLap0.resize(nbr,nbc) ;
  imG0.resize(nbr,nbc) ;

  s0.resize(dim_s0) ;

  dim_s1 = (nbr-2*bord)*(nbc-2*bord) ;
  imIx1.resize(nbr,nbc) ;
  imIy1.resize(nbr,nbc) ;
  imLap1.resize(nbr,nbc) ;

  s1.resize(dim_s1) ;

  pixInfo0 = new vpGradient[dim_s0] ;

  pixInfo1 = new vpGradient[dim_s1] ;

  Z = _Z ;
}

void
vpFeatureGradient::update(int _nbr, int _nbc, double _Z)
{
nbr = _nbr ;
nbc = _nbc ;

dim_s = (nbr-2*bord)*(nbc-2*bord) ;
imIx.resize(nbr,nbc) ;
imIy.resize(nbr,nbc) ;
imLap.resize(nbr,nbc) ;
imG.resize(nbr,nbc) ;

s.resize(dim_s) ;

dim_s0 = (nbr-2*bord)*(nbc-2*bord) ;
imIx0.resize(nbr,nbc) ;
imIy0.resize(nbr,nbc) ;
imLap0.resize(nbr,nbc) ;
imG0.resize(nbr,nbc) ;

s0.resize(dim_s0) ;

dim_s1 = (nbr-2*bord)*(nbc-2*bord) ;
imIx1.resize(nbr,nbc) ;
imIy1.resize(nbr,nbc) ;
imLap1.resize(nbr,nbc) ;

s1.resize(dim_s1) ;

Z = _Z ;


}

/*! 
  Default constructor that build a visual feature.
*/
vpFeatureGradient::vpFeatureGradient() : vpBasicFeature()
{
    init() ;
}

/*! 
  Default destructor.
*/
vpFeatureGradient::~vpFeatureGradient() 
{
  delete [] pixInfo ;
}



/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \param Z : \f$ Z \f$ value to set.
*/
void
vpFeatureGradient::set_Z(const double Z)
{
    this->Z = Z ;
    flags[0] = true;
}


/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \return The value of \f$ Z \f$.
*/
double
vpFeatureGradient::get_Z() const
{
    return Z ;
}


void
vpFeatureGradient::setCameraParameters(vpCameraParameters &_cam) 
{
  cam = _cam ;
}


/*!

  Build a luminance feature directly from the image
*/

void
vpFeatureGradient::buildFrom(vpImage<unsigned char> &I)
{
  int    l = 0;
  double Ix,Iy, Ixx, Ixy, Iyx, Iyy ;

  double px = cam.get_px() ;
  double py = cam.get_py() ;



  //  if (firstTimeIn==0)
    { 
      //  firstTimeIn=1 ;
      l =0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {	double x=0,y=0;
	      vpPixelMeterConversion::convertPoint(cam,
						   i, j,
						   y, x)  ;
	    
	      pixInfo[l].x = x;
	      pixInfo[l].y = y;

	      pixInfo[l].Z   = Z ;

	      l++;
	    }
	}   
      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    { 
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =   vpImageFilter::gaussianFilter(I,i,j) ;
	      imG[i][j] = Ix ;
	      
	    }
	}

      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    { 
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =   1 * vpImageFilter::derivativeFilterX(I,i,j);
	      Iy =   1 * vpImageFilter::derivativeFilterY(I,i,j);
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      //s[l]  = sqrt(vpMath::sqr(Ix) + vpMath::sqr(Iy)) ;
	      //std::cout<<" feature " <<s[l]*s[l]<<std::endl;
	      s[l]  = vpMath::sqr(Ix) + vpMath::sqr(Iy);
	  
	      imLap[i][j] = s[l] ;

	      pixInfo[l].Ix  = Ix;
	      pixInfo[l].Iy  = Iy;

	      Ixx =  1 * vpImageFilter::derivativeFilterX(imIx,i,j) ;
	      Ixy =  1 * vpImageFilter::derivativeFilterY(imIx,i,j) ;
	      Iyx =  1 * vpImageFilter::derivativeFilterX(imIy,i,j) ;
	      Iyy =  1 * vpImageFilter::derivativeFilterY(imIy,i,j) ;
	  
	      // Calcul de Z
	      pixInfo[l].Ixx  = Ixx;
	      pixInfo[l].Ixy  = Ixy;
	      pixInfo[l].Iyx  = Iyx;
	      pixInfo[l].Iyy  = Iyy;
	  
	      l++;
	    }
	}

    }
  /* else
    {
      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    { 
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  px*  vpImageFilter::derivativeFilterX(I,i,j) ;
	      Iy =  px*  vpImageFilter::derivativeFilterY(I,i,j) ;
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      s[l]  = ( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ; ;
	      imLap[i][j] = s[l] ;
 
	      l++;
	    }
	}
    }
  */
}

void
vpFeatureGradient::buildOrFrom(vpImage<unsigned char> &I)
{
  int    l = 0;
  double Ix,Iy, Ixx, Ixy, Iyx, Iyy ;

  double px = cam.get_px() ;
  double py = cam.get_py() ;



  //  if (firstTimeIn==0)
    {
      //  firstTimeIn=1 ;
      l =0 ;

      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =   vpImageFilter::gaussianFilter(I,i,j) ;
	      imG[i][j] = Ix ;

	    }
	}

      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =   vpImageFilter::derivativeFilterX(imG,i,j) ;
	      Iy =   vpImageFilter::derivativeFilterY(imG,i,j) ;
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)

	    {
		  double x=0,y=0;
	      vpPixelMeterConversion::convertPoint(cam,
						   i, j,
						   y, x)  ;

	      pixInfo[l].x = x;
	      pixInfo[l].y = y;

	      pixInfo[l].Z   = Z ;

	      l++;
	    }
	}
dim_s=l;
s.resize(dim_s) ;
      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      if (Iy<0)
	      {
	      s[l]  = Ix/(Iy-0.01) ;
	      }
	      else {
		  s[l]  = Ix/(Iy+0.01) ;
	      }
	      //s[l]  = vpMath::sqr(Ix) + vpMath::sqr(Iy);

	      imLap[i][j] = s[l] ;

	      pixInfo[l].Ix  = Ix;
	      pixInfo[l].Iy  = Iy;

	      Ixx =  vpImageFilter::derivativeFilterX(imIx,i,j) ;
	      Ixy =  vpImageFilter::derivativeFilterY(imIx,i,j) ;
	      Iyx =  vpImageFilter::derivativeFilterX(imIy,i,j) ;
	      Iyy =  vpImageFilter::derivativeFilterY(imIy,i,j) ;

	      // Calcul de Z
	      pixInfo[l].Ixx  = Ixx;
	      pixInfo[l].Ixy  = Ixy;
	      pixInfo[l].Iyx  = Iyx;
	      pixInfo[l].Iyy  = Iyy;

	      l++;
	    }
	}

    }
  /* else
    {
      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  px*  vpImageFilter::derivativeFilterX(I,i,j) ;
	      Iy =  px*  vpImageFilter::derivativeFilterY(I,i,j) ;
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      s[l]  = ( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ; ;
	      imLap[i][j] = s[l] ;

	      l++;
	    }
	}
    }
  */
}


void
vpFeatureGradient::buildOr1From(vpImage<unsigned char> &I)
{
  int    l = 0;
  double Ix,Iy, Ixx, Ixy, Iyx, Iyy ;

  double px = cam.get_px() ;
  double py = cam.get_py() ;
  vpImagePoint ip;



  //  if (firstTimeIn==0)
    {
      //  firstTimeIn=1 ;
      l =0 ;

      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =   vpImageFilter::gaussianFilter(I,i,j) ;
	      imG[i][j] = Ix ;

	    }
	}

      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =   vpImageFilter::derivativeFilterX(imG,i,j) ;
	      Iy =   vpImageFilter::derivativeFilterY(imG,i,j) ;
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)

	    {
		  double x=0,y=0;
	      vpPixelMeterConversion::convertPoint(cam,
						   i, j,
						   y, x)  ;

	      pixInfo[l].x = x;
	      pixInfo[l].y = y;

	      pixInfo[l].Z   = Z ;

	      l++;
	    }
	}
dim_s=l;
s.resize(dim_s) ;
      l= 0 ;
      int m=0;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      if ((vpMath::sqr(Ix)+vpMath::sqr(Iy))>200)
	      {
	      s[l]  = atan(Ix/Iy) ;
	      ip.set_i(i);
	      ip.set_j(j);
	      vpDisplay::displayPoint(I,i,j,vpColor::red);
	      //cout<<"ssss  "<<s[l]<<"  "<<j<<endl;
	      m++;
	      }
	      else {
		  s[l]  = 0 ;
	      }
	      //cout<<"ssss"<<s[l]<<endl;
	      //s[l]  = vpMath::sqr(Ix) + vpMath::sqr(Iy);

	      imLap[i][j] = s[l] ;

	      pixInfo[l].Ix  = Ix;
	      pixInfo[l].Iy  = Iy;

	      Ixx =  vpImageFilter::derivativeFilterX(imIx,i,j) ;
	      Ixy =  vpImageFilter::derivativeFilterY(imIx,i,j) ;
	      Iyx =  vpImageFilter::derivativeFilterX(imIy,i,j) ;
	      Iyy =  vpImageFilter::derivativeFilterY(imIy,i,j) ;

	      // Calcul de Z
	      pixInfo[l].Ixx  = Ixx;
	      pixInfo[l].Ixy  = Ixy;
	      pixInfo[l].Iyx  = Iyx;
	      pixInfo[l].Iyy  = Iyy;

	      l++;
	    }
	}
//cout<<"mmm "<<m<<endl;
    }
  /* else
    {
      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  px*  vpImageFilter::derivativeFilterX(I,i,j) ;
	      Iy =  px*  vpImageFilter::derivativeFilterY(I,i,j) ;
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      s[l]  = ( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ; ;
	      imLap[i][j] = s[l] ;

	      l++;
	    }
	}
    }
  */
}



void
vpFeatureGradient::buildTh0From(vpImage<unsigned char> &I, double th)
{
  int    l = 0;
  double Ix,Iy, Ixx, Ixy, Iyx, Iyy ;

  double px = cam.get_px() ;
  double py = cam.get_py() ;



  //  if (firstTimeIn==0)
    {
      //  firstTimeIn=1 ;
      l =0 ;

      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =   vpImageFilter::gaussianFilter(I,i,j) ;
	      imG[i][j] = Ix ;

	    }
	}

      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =   vpImageFilter::derivativeFilterX(imG,i,j) ;
	      Iy =   vpImageFilter::derivativeFilterY(imG,i,j) ;
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)

	    {
		  if(vpMath::sqr(imIx[i][j])+vpMath::sqr(imIy[i][j])>th)
		  {
		  double x=0,y=0;
	      vpPixelMeterConversion::convertPoint(cam,
						   i, j,
						   y, x)  ;

	      pixInfo[l].x = x;
	      pixInfo[l].y = y;

	      pixInfo[l].Z   = Z ;

	      l++;
		  }
	    }
	}
dim_s=l;
s.resize(dim_s) ;
      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
		  if(vpMath::sqr(imIx[i][j])+vpMath::sqr(imIy[i][j])>th)
		  {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      s[l]  = sqrt( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ; ;
	      //s[l]  = vpMath::sqr(Ix) + vpMath::sqr(Iy);

	      imLap[i][j] = s[l] ;

	      pixInfo[l].Ix  = Ix;
	      pixInfo[l].Iy  = Iy;

	      Ixx =  vpImageFilter::derivativeFilterX(imIx,i,j) ;
	      Ixy =  vpImageFilter::derivativeFilterY(imIx,i,j) ;
	      Iyx =  vpImageFilter::derivativeFilterX(imIy,i,j) ;
	      Iyy =  vpImageFilter::derivativeFilterY(imIy,i,j) ;

	      // Calcul de Z
	      pixInfo[l].Ixx  = Ixx;
	      pixInfo[l].Ixy  = Ixy;
	      pixInfo[l].Iyx  = Iyx;
	      pixInfo[l].Iyy  = Iyy;

	      l++;
		  }
	    }
	}

    }
  /* else
    {
      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  px*  vpImageFilter::derivativeFilterX(I,i,j) ;
	      Iy =  px*  vpImageFilter::derivativeFilterY(I,i,j) ;
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      s[l]  = ( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ; ;
	      imLap[i][j] = s[l] ;

	      l++;
	    }
	}
    }
  */
}

void
vpFeatureGradient::computeGradients(vpImage<unsigned char> &I0)
{
double Ix0,Iy0;
	for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix0 =   vpImageFilter::gaussianFilter(I0,i,j) ;
	      imG0[i][j] = Ix0 ;

	    }
	}

    for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix0 =   vpImageFilter::derivativeFilterX(imG0,i,j) ;
	      Iy0 =   vpImageFilter::derivativeFilterY(imG0,i,j) ;
		  /*Ix0 =   vpImageFilter::derivativeFilterX(I0,i,j) ;
		  Iy0 =   vpImageFilter::derivativeFilterY(I0,i,j) ;*/
	      imIx0[i][j] = Ix0 ;
	      imIy0[i][j] = Iy0 ;
	    }
	}
}

void
vpFeatureGradient::buildThFrom(vpImage<unsigned char> &I0,vpImage<unsigned char> &I1, double th)
{
  int    l = 0;
  double Ix0,Iy0, Ixx0, Ixy0, Iyx0, Iyy0 ;
  double Ix1,Iy1, Ixx1, Ixy1, Iyx1, Iyy1 ;
  double x=0,y=0;

  double px = cam.get_px() ;
  double py = cam.get_py() ;

  //  if (firstTimeIn==0)
    {
      //  firstTimeIn=1 ;

      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix1 =   vpImageFilter::derivativeFilterX(I1,i,j) ;
	      Iy1 =   vpImageFilter::derivativeFilterY(I1,i,j) ;
	      imIx1[i][j] = Ix1 ;
	      imIy1[i][j] = Iy1 ;
	    }
	}

/*      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)

	    {
		  if(vpMath::sqr(imIx0[i][j])+vpMath::sqr(imIy0[i][j])>th || vpMath::sqr(imIx1[i][j])+vpMath::sqr(imIy1[i][j])>th)
		  {
	      l++;
		  }
	    }
	}
*/
      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
		  if((vpMath::sqr(imIx0[i][j])+vpMath::sqr(imIy0[i][j]))>th || (vpMath::sqr(imIx1[i][j])+vpMath::sqr(imIy1[i][j]))>th)
		  {
			  	      vpPixelMeterConversion::convertPoint(cam,
			  						   i, j,
			  						   y, x)  ;
		      pixInfo0[l].x = x;
		      pixInfo0[l].y = y;

		      pixInfo0[l].Z   = Z ;

		      /*pixInfo1[l].x = x;
		      pixInfo1[l].y = y;

		      pixInfo1[l].Z   = Z ;*/

	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix0 =  imIx0[i][j] ;
	      Iy0 =  imIy0[i][j] ;
	      Ix1 =  imIx1[i][j] ;
	      Iy1 =  imIy1[i][j] ;
	      pixInfo0[l].I  =  I0[i][j] ;
	      pixInfo1[l].I  =  I1[i][j] ;
	      //s0[l]  = sqrt( vpMath::sqr(Ix0) + vpMath::sqr(Iy0) ) ; ;
	      //s1[l]  = sqrt( vpMath::sqr(Ix1) + vpMath::sqr(Iy1) ) ; ;
	      s0[l]  = vpMath::sqr(Ix0)*vpMath::sqr(Ix0) + vpMath::sqr(Iy0)*vpMath::sqr(Iy0);
	      s1[l]  = vpMath::sqr(Ix1)*vpMath::sqr(Ix1) + vpMath::sqr(Iy1)*vpMath::sqr(Iy1);

	      /*imLap0[i][j] = s0[l] ;
	      imLap1[i][j] = s1[l] ;*/

	      pixInfo0[l].Ix  = Ix0;
	      pixInfo0[l].Iy  = Iy0;
	      /*pixInfo1[l].Ix  = Ix1;
	      pixInfo1[l].Iy  = Iy1;*/

	      Ixx0 =  vpImageFilter::derivativeFilterX(imIx0,i,j) ;
	      Ixy0 =  vpImageFilter::derivativeFilterY(imIx0,i,j) ;
	      Iyx0 =  vpImageFilter::derivativeFilterX(imIy0,i,j) ;
	      Iyy0 =  vpImageFilter::derivativeFilterY(imIy0,i,j) ;

	      // Calcul de Z
	      pixInfo0[l].Ixx  = Ixx0;
	      pixInfo0[l].Ixy  = Ixy0;
	      pixInfo0[l].Iyx  = Iyx0;
	      pixInfo0[l].Iyy  = Iyy0;

	      /*Ixx1 =  vpImageFilter::derivativeFilterX(imIx1,i,j) ;
	      Ixy1 =  vpImageFilter::derivativeFilterY(imIx1,i,j) ;
	      Iyx1 =  vpImageFilter::derivativeFilterX(imIy1,i,j) ;
	      Iyy1 =  vpImageFilter::derivativeFilterY(imIy1,i,j) ;*/

	      // Calcul de Z
	      /*pixInfo1[l].Ixx  = Ixx1;
	      pixInfo1[l].Ixy  = Ixy1;
	      pixInfo1[l].Iyx  = Iyx1;
	      pixInfo1[l].Iyy  = Iyy1;*/

	      l++;
		  }
	    }
	}
      dim_s0=l;
      dim_s1=l;
      //cout<< " l "<<l<<endl;
s0.resize(dim_s0,false);
s1.resize(dim_s0,false);
    }
  /* else
    {
      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  px*  vpImageFilter::derivativeFilterX(I,i,j) ;
	      Iy =  px*  vpImageFilter::derivativeFilterY(I,i,j) ;
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      s[l]  = ( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ; ;
	      imLap[i][j] = s[l] ;

	      l++;
	    }
	}
    }
  */
}



/*void
vpFeatureGradient::buildTh1From(vpImage<unsigned char> &I1, vpBasicFeature &s0, vpGradient &pixinfo0, double th)
{
  int    l = 0;
  double Ix1,Iy1, Ixx1, Ixy1, Iyx1, Iyy1 ;

  double px = cam.get_px() ;
  double py = cam.get_py() ;

  //  if (firstTimeIn==0)
    {
      //  firstTimeIn=1 ;
      l =0 ;

      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j=3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix1 =   vpImageFilter::derivativeFilterX(I1,i,j) ;
	      Iy1 =   vpImageFilter::derivativeFilterY(I1,i,j) ;
	      imIx1[i][j] = Ix1 ;
	      imIy1[i][j] = Iy1 ;
	    }
	}

      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
		  if(vpMath::sqr(imIx1[i][j])+vpMath::sqr(imIy1[i][j])>th)
		  {

			  double x=0,y=0;
		      vpPixelMeterConversion::convertPoint(cam,
							   i, j,
							   y, x)  ;

		      pixInfo1[l].x = x;
		      pixInfo1[l].y = y;

		      pixInfo1[l].Z   = Z ;
		  }
	    }
	}



dim_s1=l;
s1.resize(dim_s1) ;
      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
		  if(vpMath::sqr(imIx1[i][j])+vpMath::sqr(imIy1[i][j])>th)
		  {

	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix1 =  imIx1[i][j] ;
	      Iy1 =  imIy1[i][j] ;
	      pixInfo1[l].I  =  I1[i][j] ;
	      s1[l]  = sqrt( vpMath::sqr(Ix1) + vpMath::sqr(Iy1) ) ; ;
	      //s[l]  = vpMath::sqr(Ix) + vpMath::sqr(Iy);

	      imLap1[i][j] = s1[l] ;

	      pixInfo1[l].Ix  = Ix;
	      pixInfo1[l].Iy  = Iy;

	      Ixx1 =  vpImageFilter::derivativeFilterX(imIx1,i,j) ;
	      Ixy1 =  vpImageFilter::derivativeFilterY(imIx1,i,j) ;
	      Iyx1 =  vpImageFilter::derivativeFilterX(imIy1,i,j) ;
	      Iyy1 =  vpImageFilter::derivativeFilterY(imIy1,i,j) ;

	      // Calcul de Z
	      pixInfo1[l].Ixx1  = Ixx1;
	      pixInfo1[l].Ixy1  = Ixy1;
	      pixInfo1[l].Iyx1  = Iyx1;
	      pixInfo1[l].Iyy1  = Iyy1;

	      for (int l0=0;l0<dim_s0;l0++)
	      {
	    	if (pixInfo1[l].x=pixInfo0[l].x && )
	      }

	      l++;
		  }
	    }
	}

    }
  /* else
    {
      for (int i=3; i < nbr-3 ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = 3 ; j < nbc-3; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  px*  vpImageFilter::derivativeFilterX(I,i,j) ;
	      Iy =  px*  vpImageFilter::derivativeFilterY(I,i,j) ;
	      imIx[i][j] = Ix ;
	      imIy[i][j] = Iy ;
	    }
	}

      l= 0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {
	      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	      Ix =  imIx[i][j] ;
	      Iy =  imIy[i][j] ;
	      pixInfo[l].I  =  I[i][j] ;
	      s[l]  = ( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ; ;
	      imLap[i][j] = s[l] ;

	      l++;
	    }
	}
    }
  */
//}



/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
void
vpFeatureGradient::interactionOr(vpMatrix &L)
{
  double x,y,Ix,Iy,Ixx,Ixy, Iyx, Iyy, Zinv;

  L.resize(dim_s,6) ;

  for(int m = 0; m< L.getRows(); m++)
    {
      Ix = pixInfo[m].Ix;
      Iy = pixInfo[m].Iy;
      Ixx = pixInfo[m].Ixx;
      Ixy = pixInfo[m].Ixy;
      Iyx = pixInfo[m].Iyx;
      Iyy = pixInfo[m].Iyy;

      x = pixInfo[m].x ;
      y = pixInfo[m].y ;
      Zinv =  1 / pixInfo[m].Z;

      // equation 12 icra '07
      /*double A = -( Ixx*Ix+Iyx*Iy );
      double B = -( Ixy*Ix+Iyy*Iy );*/
      double A = -( Ixx*Iy-Iyx*Ix )/(vpMath::sqr(Iy) + 0.01);
      double B = -( Ixy*Iy-Iyy*Ix )/(vpMath::sqr(Iy) + 0.01);
      {
	L[m][0] = -A * Zinv;
	L[m][1] = -B * Zinv;
	L[m][2] = (A*x+B*y)*Zinv;
	L[m][3] = A*x*y + B*(1+y*y);
	L[m][4] = -A*(1+x*x) - B*x*y;
	L[m][5]  = A*y - B*x;
      }
    }
}

void
vpFeatureGradient::interactionOr1(vpMatrix &L)
{

	double A,B,x,y,Ix,Iy,Ixx,Ixy, Iyx, Iyy, Zinv;

  L.resize(dim_s,6) ;

  for(int m = 0; m< L.getRows(); m++)
    {
      Ix = pixInfo[m].Ix;
      Iy = pixInfo[m].Iy;
      Ixx = pixInfo[m].Ixx;
      Ixy = pixInfo[m].Ixy;
      Iyx = pixInfo[m].Iyx;
      Iyy = pixInfo[m].Iyy;

      x = pixInfo[m].x ;
      y = pixInfo[m].y ;
      Zinv =  1 / pixInfo[m].Z;

      // equation 12 icra '07
      /*double A = -( Ixx*Ix+Iyx*Iy );
      double B = -( Ixy*Ix+Iyy*Iy );*/
      if (vpMath::sqr(Iy)+vpMath::sqr(Ix)>100)
      {
             A = -(( Ixx*Iy-Iyx*Ix )/(vpMath::sqr(Iy)))*(1/(1+(vpMath::sqr(Ix)/(vpMath::sqr(Iy)))));
             B = -(( Ixy*Iy-Iyy*Ix )/(vpMath::sqr(Iy)))*(1/(1+(vpMath::sqr(Ix)/(vpMath::sqr(Iy)))));
      }
      else {
             A=0;
             B=0;
      }
      {
	L[m][0] = -A * Zinv;
	L[m][1] = -B * Zinv;
	L[m][2] = (A*x+B*y)*Zinv;
	L[m][3] = A*x*y + B*(1+y*y);
	L[m][4] = -A*(1+x*x) - B*x*y;
	L[m][5]  = A*y - B*x;
      }
    }
}


void
vpFeatureGradient::interaction(vpMatrix &L)
{
  double x,y,Ix,Iy,Ixx,Ixy, Iyx, Iyy, Zinv;

  L.resize(dim_s,6) ;

  for(int m = 0; m< L.getRows(); m++)
    {
      Ix = pixInfo[m].Ix;
      Iy = pixInfo[m].Iy;
      Ixx = pixInfo[m].Ixx;
      Ixy = pixInfo[m].Ixy;
      Iyx = pixInfo[m].Iyx;
      Iyy = pixInfo[m].Iyy;

      x = pixInfo[m].x ;
      y = pixInfo[m].y ;
      Zinv =  1 / pixInfo[m].Z;

      // equation 12 icra '07
      double A = -2*( Ixx*Ix+Iyx*Iy );
      double B = -2*( Ixy*Ix+Iyy*Iy );
      //double A = -( Ixx*Ix+Ixy*Iy )/(sqrt( vpMath::sqr(Ix) + vpMath::sqr(Iy) + 0.1));
      //double B = -( Iyx*Ix+Iyy*Iy )/(sqrt( vpMath::sqr(Ix) + vpMath::sqr(Iy) + 0.1));
      {
	L[m][0] = -A * Zinv;
	L[m][1] = -B * Zinv;
	L[m][2] = (A*x+B*y)*Zinv;
	L[m][3] = A*x*y + B*(1+y*y);
	L[m][4] = -A*(1+x*x) - B*x*y;
	L[m][5]  = A*y - B*x;
      }
    }
}

void
vpFeatureGradient::interactionTh(vpMatrix &L)
{
  double x,y,Ix,Iy,Ixx,Ixy, Iyx, Iyy, Zinv;

  L.resize(dim_s0,6) ;

  for(int m = 0; m< L.getRows(); m++)
    {
      Ix = pixInfo0[m].Ix;
      Iy = pixInfo0[m].Iy;
      Ixx = pixInfo0[m].Ixx;
      Ixy = pixInfo0[m].Ixy;
      Iyx = pixInfo0[m].Iyx;
      Iyy = pixInfo0[m].Iyy;

      x = pixInfo0[m].x ;
      y = pixInfo0[m].y ;
      Zinv =  1 / pixInfo0[m].Z;

      // equation 12 icra '07
      double A = -4*( Ixx*Ix+Iyx*Iy );
      double B = -4*( Ixy*Ix+Iyy*Iy );
      /*double A = -( Ixx*Ix+Ixy*Iy )/(sqrt( vpMath::sqr(Ix) + vpMath::sqr(Iy) + 0.1));
      double B = -( Iyx*Ix+Iyy*Iy )/(sqrt( vpMath::sqr(Ix) + vpMath::sqr(Iy) + 0.1));*/
      {
	L[m][0] = -A * Zinv;
	L[m][1] = -B * Zinv;
	L[m][2] = (A*x+B*y)*Zinv;
	L[m][3] = 10*(A*x*y + B*(1+y*y));
	L[m][4] = 10*(-A*(1+x*x) - B*x*y);
	L[m][5] = 10*(A*y - B*x);
      }
    }
}


/*!
  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
vpMatrix  vpFeatureGradient::interaction(const int select)
{
  static vpMatrix L  ;
  interaction(L) ;
  return L ;
}


/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.

*/

/*void
vpFeatureGradient::updateFeatures1(const vpBasicFeature &s1, vpGradient &pixinfo1)
{
int m=0;
dim_s1=s1.dimension_s();
	for (int i =0 ; i < dim_s1 ; i++)
	{
for (int j=0;j<dim_s;j++)
{

}
	}
}
*/
void
vpFeatureGradient::error(const vpBasicFeature &s_star,
			  vpColVector &e)
{
  e.resize(dim_s) ;

  for (int i =0 ; i < dim_s ; i++)
    {
      e[i] = s[i] - s_star[i] ;
    }
  
}

void
vpFeatureGradient::errorTh(
			  vpColVector &e)
{
  e.resize(dim_s0) ;

  for (int i =0 ; i < dim_s0 ; i++)
    {
      e[i] = s1[i] - s0[i] ;
    }

}



/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.

*/
vpColVector
vpFeatureGradient::error(const vpBasicFeature &s_star,
		      const int select)
{
  static vpColVector e ;
  
  error(s_star,e) ;
  
  return e ;

}




/*!

  Not implemented.

 */
void
vpFeatureGradient::print(const int select ) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
 }



/*!

  Not implemented.

 */
void
vpFeatureGradient::display(const vpCameraParameters &cam,
			vpImage<unsigned char> &I,
			vpColor color, unsigned int thickness) const
{
 static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Not implemented.

 */
void
vpFeatureGradient::display(const vpCameraParameters &cam,
                        vpImage<vpRGBa> &I,
                        vpColor color, unsigned int thickness) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}


/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureGradient s;
  s_star = s.duplicate(); // s_star is now a vpFeatureGradient
  \endcode

*/
vpFeatureGradient *vpFeatureGradient::duplicate() const
{
  vpFeatureGradient *feature = new vpFeatureGradient ;
  return feature ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
