

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

  s.resize(dim_s) ;
  
  pixInfo = new vpGradient[dim_s] ;

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


  if (firstTimeIn==0)
    { 
      firstTimeIn=1 ;
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
    }

  for (int i=3; i < nbr-3 ; i++)
    {
      //   cout << i << endl ;
      for (int j = 3 ; j < nbc-3; j++)
	{ 
	  // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	  Ix =  px*vpImageFilter::derivativeFilterX(I,i,j) ;
	  Iy =  py*vpImageFilter::derivativeFilterY(I,i,j) ;
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
	  Ix =  imIx[i][j];
	  Iy =  imIy[i][j];

	  Ixx = vpImageFilter::derivativeFilterX(imIx,i,j) ;
	  Ixy = vpImageFilter::derivativeFilterY(imIx,i,j) ;
	  Iyx = vpImageFilter::derivativeFilterX(imIy,i,j) ;
	  Iyy = vpImageFilter::derivativeFilterY(imIy,i,j) ;

	  // Calcul de Z
	  
	  pixInfo[l].I  =  I[i][j] ;
	  s[l]  =  vpMath::sqr(Ix) + vpMath::sqr(Iy) ; ;
	  pixInfo[l].Ix  = Ix;
	  pixInfo[l].Iy  = Iy;
	  pixInfo[l].Ixx  = Ixx;
	  pixInfo[l].Ixy  = Ixy;
	  pixInfo[l].Iyx  = Iyx;
	  pixInfo[l].Iyy  = Iyy;
	  
	  l++;
	}
    }

}




/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
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
      double A = 2*(Ixx*Ix+Iyx*Iy) ;
      double B = 2*(Ixy*Ix+Iyy*Iy) ;
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
