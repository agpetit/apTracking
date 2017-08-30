#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImageFilter.h>
#include <visp/vpPoint.h>
#include <visp/vpFeatureDisplay.h>


#include "apFeatureGradientOrientation.h"


using namespace std ;


/*!
  \file vpFeatureLuminance.cpp
  \brief Class that defines the image luminance visual feature

  for more details see
  C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/



/*!
  Initialize the memory space requested for vpFeatureLuminance visual feature.
*/
void
apFeatureGradientOrientation::init()
{

    //default value Z (1 meters)
    Z = 1;

    firstTimeIn =0 ;

}


void
apFeatureGradientOrientation::init(int _nbr, int _nbc, double _Z)
{
  init() ;

  nbr = _nbr ;
  nbc = _nbc ;
  // number of feature = nb column x nb lines in the images
  dim_s = (nbr-2*bord)*(nbc-2*bord) ;

  s.resize(dim_s) ;
  OriInfo = new apOrientation[dim_s] ;
  
  
  Z = _Z ;
}

/*! 
  Default constructor that build a visual feature.
*/
apFeatureGradientOrientation::apFeatureGradientOrientation() : vpBasicFeature()
{
    nbParameters = 1;
    dim_s = 0 ;
    bord = 10 ;

    init() ;
}

/*! 
  Default destructor.
*/
apFeatureGradientOrientation::~apFeatureGradientOrientation()
{

}



/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \param Z : \f$ Z \f$ value to set.
*/

void
apFeatureGradientOrientation::update(int _nbr, int _nbc, double _Z)
{
nbr = _nbr ;
nbc = _nbc ;

dim_s = (nbr-2*bord)*(nbc-2*bord) ;
s.resize(dim_s) ;
Z = _Z ;
}


void
apFeatureGradientOrientation::set_Z(const double Z)
{
    this->Z = Z ;
}


/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \return The value of \f$ Z \f$.
*/
double
apFeatureGradientOrientation::get_Z() const
{
    return Z ;
}


void
apFeatureGradientOrientation::setCameraParameters(vpCameraParameters &_cam)
{
  cam = _cam ;
}


/*!

  Build a luminance feature directly from the image
*/

void
apFeatureGradientOrientation::buildFrom(vpMatrix &Ori, vpImage<vpRGBa> &Ic, vpMatrix &Zcoord, vpHomogeneousMatrix &cMo, vpImage<unsigned char> &Iv)
{
  int    l = 0;
  double Ac,Bc,Cc,Dc,x,y;
  double rho,theta;
  double px = cam.get_px();
  double py = cam.get_py();
  int jc = cam.get_u0() ;
  int ic = cam.get_v0() ;
  int i1,j1;

vpFeatureLine Oriline;
vpFeatureLine Oriline1;
vpPoint Pc;
vpRotationMatrix R;
cMo.extract(R);
vpColVector Normo(3);
vpColVector Normc(3);
vpRGBa col;
vpColVector Pc0(3);
vpColVector ip0(2);
vpImagePoint imp0;
vpImagePoint imp1;



 // if (firstTimeIn==0)
    {
 //     firstTimeIn=1 ;
      l =0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {	double x=0,y=0;
	      x=(j-jc)/px;
	      y=(i-ic)/py;
	      Pc.set_X(x*Zcoord[i][j]);
	      Pc.set_Y(y*Zcoord[i][j]);
	      Pc.set_Z(Zcoord[i][j]);
	      theta=Ori[i][j];
          col=Ic[i][j];
          Normo[0]=(double)(col.R/255);
          Normo[1]=-(double)(col.G/255);
          Normo[2]=(double)(col.B/255);
          Normc=R*Normo;
          if(Normc[0]==0 && Normc[1]==0 && Normc[2]==0)
          {Normc[0]=0;
          Normc[1]=0;
          Normc[2]=1;
          theta=M_PI/2;
          }
          Pc0[0]=x*Zcoord[i][j]+Normc[0]/40;
          Pc0[1]=y*Zcoord[i][j]+Normc[1]/40;
          Pc0[2]=Zcoord[i][j]+Normc[2]/40;
          Pc.projection(Pc0,ip0);
          j1=ip0[0]*px+jc;
          i1=ip0[1]*py+ic;
          /*if (theta<M_PI/2 || theta>M_PI/2){
        	  if ((double)i*j/10-floor(i*j/10)<0.05 && theta>0 && theta<M_PI){
        	      vpDisplay::displayArrow(Iv,i,j,i1,j1,vpColor::white,4,2,1);
        		  }
          }*/
          rho= x*cos(theta)+y*sin(theta);
          Ac=Normc[0];
          Bc=Normc[1];
          Cc=Normc[2];
          Dc=-Ac*Pc.get_X()-Bc*Pc.get_Y()-Cc*Pc.get_Z();
	      //s[2*l]=rho;
          OriInfo[l].i=i;
          OriInfo[l].j=j;
          OriInfo[l].x=x;
          OriInfo[l].y=y;
          OriInfo[l].A=Ac;
          OriInfo[l].B=Bc;
          OriInfo[l].C=Cc;
          OriInfo[l].D=Dc;
          OriInfo[l].rho=rho;
          OriInfo[l].theta=theta;
	      //s[2*l+1]=theta;
          s[l]=theta;
          //Oriline.buildFrom(rho,theta,Ac,Bc,Cc,Dc);
          //Orilines.addRight(Oriline);
	      l++;
	    }
	}
      dim_s=l;
      //cout<<dim_s<<endl;
    }


}

void
apFeatureGradientOrientation::buildFrom(vpMatrix &Ori, vpImage<unsigned char> &Iv)
{
  int    l = 0;
  double rho,theta;
  double px = cam.get_px() ;
  double py = cam.get_py() ;
  vpFeatureLine Oriline;
  int jc = cam.get_u0() ;
  int ic = cam.get_v0() ;

  //if (firstTimeIn==0)
      //firstTimeIn=1 ;
      l =0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {	double x=0,y=0;
	      x=(j-jc)/px;
	      y=(i-ic)/py;
	      theta=Ori[i][j];
          rho= x*cos(theta)+y*sin(theta);
	      //s[2*l]=theta;
          s[l]=theta;
          OriInfo[l].rho=rho;
          OriInfo[l].theta=theta;
          Oriline.buildFrom(rho,theta);
          if (theta<M_PI/2 || theta>M_PI/2){
        	  if ((double)i*j/1-floor(i*j/1)<0.05 && theta>0 && theta<M_PI){
        	      vpDisplay::displayCross(Iv,i,j,2,vpColor::red,1);
          Oriline.display(cam,Iv,vpColor::yellow,1);
        		  }
          }
	      //s[2*l+1]=theta;
          //Oriline.buildFrom(rho,theta);
          //Orilines.addRight(Oriline);
	      l++;
	    }
	}
      //cout<<s[150*400]<<endl;
      dim_s=l;
      //cout<<dim_s<<endl;


}



/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
void
apFeatureGradientOrientation::interaction(vpMatrix &L, vpImage<unsigned char> &Iv)
{

  L.resize(dim_s,6) ;
//Orilines.front();
vpFeatureLine Oriline;
vpFeatureLine Oriline1;
double rho,theta, A,B,C,D,x,y;
int i,j;
vpMatrix H;
int m=0;
int k=0;
//cout<<dim_s<<endl;
 // while(!Orilines.outside())
for (m=0;m<dim_s;m++)
    {
      //Oriline=Orilines.value();
	  A=OriInfo[m].A;
	  B=OriInfo[m].B;
	  C=OriInfo[m].C;
	  D=OriInfo[m].D;
	  i=OriInfo[m].i;
	  j=OriInfo[m].j;
	  x=OriInfo[m].x;
	  y=OriInfo[m].y;
	  rho=OriInfo[m].rho;
	  theta=OriInfo[m].theta;
      Oriline.buildFrom(rho,theta,A,B,C,D);
      if (theta<M_PI/2 || theta>M_PI/2){
    	  if ((double)m/1-floor(m/1)<0.001 && theta>0 && theta<M_PI){
    	      vpDisplay::displayCross(Iv,i,j,2,vpColor::red,1);
      Oriline.display(cam,Iv,vpColor::yellow,1);
    		  }
      }
      H=Oriline.interaction(vpFeatureLine::selectTheta());
      {

    L[m][0] = H[0][0];
    L[m][1] = H[0][1];
    L[m][2] = H[0][2];
    L[m][3] = H[0][3];
    L[m][4] = H[0][4];
    L[m][5] = H[0][5];

 /*   L[2*m][0] = H[0][0];
	L[2*m][1] = H[0][1];
	L[2*m][2] = H[0][2];
	L[2*m][3] = H[0][3];
	L[2*m][4] = H[0][4];
	L[2*m][5] = H[0][5];


	L[2*m+1][0] = H[1][0];
	L[2*m+1][1] = H[1][1];
	L[2*m+1][2] = H[1][2];
	L[2*m+1][3] = H[1][3];
	L[2*m+1][4] = H[1][4];
	L[2*m+1][5] = H[1][5];*/
      }
      //Orilines.next();
     // m++;
    }
}

/*!
  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
vpMatrix  apFeatureGradientOrientation::interaction(const int /* select */)
{
  /*static vpMatrix L  ;
  interaction(L) ;
  return L ;*/
}


/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.
  \param e : Error between the current and the desired features.

*/
vpList< vpFeatureLine > apFeatureGradientOrientation::getFeatureLines()
{
	//return Orilines;
}

void
apFeatureGradientOrientation::error(const vpBasicFeature &s_star,
			  vpColVector &e)
{
  /*e.resize(dim_s);
  vpList< vpFeatureLine > Orilines_star = s_star.getFeatureLines();
  Orilines.front();
  Orilines_star.front();
  vpFeatureLine Oriline;
  for (int i =0 ; i < dim_s ; i++)
    {
	  Oriline=Orilines.value();
	  e[i]=Oriline.error(Orilines_star.value(),vpFeatureLine::selectTheta());
	  Orilines.next();
	  Orilines_star.next();
    }*/
	cout<<dim_s<<endl;
	e.resize(dim_s);
	for (int i =0 ; i < dim_s ; i++)
	    {
		if (s[i]==M_PI/2 || s_star[i]==M_PI/2)
	      {e[i] = 0;

	      }
		else {
			e[i] = s[i] - s_star[i] ;
			//cout<<e[i]<<endl;
		}
	    }

}



/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.
  \param select : Not used.

*/
vpColVector
apFeatureGradientOrientation::error(const vpBasicFeature &s_star,
			  const int /* select */)
{
  static vpColVector e ;
  
  error(s_star, e) ;
  
  return e ;

}




/*!

  Not implemented.

 */
void
apFeatureGradientOrientation::print(const int /* select */) const
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
apFeatureGradientOrientation::display(const vpCameraParameters & /* cam */,
			    vpImage<unsigned char> & /* I */,
			    vpColor /* color */,  unsigned int /* thickness */) const
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
apFeatureGradientOrientation::display(const vpCameraParameters & /* cam */,
			    vpImage<vpRGBa> & /* I */,
			    vpColor /* color */, unsigned int /* thickness */) const
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
  vpFeatureLuminance s;
  s_star = s.duplicate(); // s_star is now a vpFeatureLuminance
  \endcode

*/
apFeatureGradientOrientation *apFeatureGradientOrientation::duplicate() const
{
  apFeatureGradientOrientation *feature = new apFeatureGradientOrientation ;
  return feature ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
