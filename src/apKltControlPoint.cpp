/*
 * apKltControlPoint.cpp
 *
 *  Created on: Apr 24, 2013
 *      Author: agpetit
 */

#include "apKltControlPoint.h"
#include <visp/vpTrackingException.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpRobust.h>

apKltControlPoint::apKltControlPoint()
{
	// TODO Auto-generated constructor stub
	  /*icpoint=NULL;
	  icpoint_curr=NULL;
	  cpoint=NULL;*/
	  norm.resize(3);
	  //normw.resize(3);
	  L.resize(2,6);
	  error.resize(2);
	  //featurepoint.init();
	  valid = true;

}

apKltControlPoint::~apKltControlPoint() {
	// TODO Auto-generated destructor stub
	  /*if (icpoint != NULL) delete icpoint ;
	  if (icpoint_curr != NULL) delete icpoint_curr ;
	  if (cpoint != NULL) delete cpoint ;*/
}

void
apKltControlPoint::setMovingEdge(vpMe _me)
{
  //me = _me ;

}

void
apKltControlPoint::setCameraParameters(vpCameraParameters _cam)
{
  cam = _cam ;

}

/*!
  Build a 3D plane thanks the 3D coordinate of the control point and the normal vector to the surface

  \param plane : The vpPlane instance used to store the computed plane equation.
*/

void
apKltControlPoint::buildPlane(vpPoint &pointn, vpColVector &normal,vpPlane &plane)
{
  //Equation of the plane is given by:
  double A = normal[0];
  double B = normal[1];
  double C = normal[2];
  //double D=-(A*(double)cpoint->get_oX()+B*(double)cpoint->get_oY()+C*(double)cpoint->get_oZ());
  double D=-(A*(double)pointn.get_oX()+B*pointn.get_oY()+C*(double)pointn.get_oZ());

  double normp =  sqrt(A*A+B*B+C*C) ;
  plane.setA(A/normp);
  plane.setB(B/normp);
  plane.setC(C/normp);
  plane.setD(D/normp);
  /*plane.setA(A);
  plane.setB(B);
  plane.setC(C);
  plane.setD(D);*/
}

void
apKltControlPoint::initPoint(int y, int x, double &Z,  vpColVector &normo, vpHomogeneousMatrix &cMo)
{
/*icpoint =new vpImagePoint;
cpoint = new vpPoint;*/
vpRotationMatrix R;
vpHomogeneousMatrix oMc;
oMc=cMo.inverse();
cMo.extract(R);
double px = cam.get_px();
double py = cam.get_py();
int jc = cam.get_u0();
int ic = cam.get_v0();
icpoint.set_i(y);
icpoint.set_j(x);
double xp,yp;
xp=(x-jc)/px;
yp=(y-ic)/py;
cpoint.setWorldCoordinates(xp*Z,yp*Z,Z);
cpoint.changeFrame(oMc);
cpointo.setWorldCoordinates(cpoint.get_X(),cpoint.get_Y(),cpoint.get_Z());
normw=normo;
norm=R*normo;
icpoint0 = icpoint;

// initialisation of the value for the computation in SE3
vpPlane plane;//(normo[0], normo[1], normo[2]);
buildPlane(cpointo,normo,plane);
d0 = plane.getD();
N = plane.getNormal();

N.normalize();
N_cur = N;
//invd0 = 1.0 / d0;
invd0 = 1.0 / Z;
R0 = R;
Z0 = Z;

}

void
apKltControlPoint::buildPoint(int y, int x, double &Z,  vpColVector &normo, vpHomogeneousMatrix &cMo)
{
/*icpoint =new vpImagePoint;
cpoint = new vpPoint;*/
vpRotationMatrix R;
vpHomogeneousMatrix oMc;
oMc=cMo.inverse();
cMo.extract(R);
double px = cam.get_px();
double py = cam.get_py();
int jc = cam.get_u0();
int ic = cam.get_v0();
icpoint.set_i(y);
icpoint.set_j(x);
double xp,yp;
xp=(x-jc)/px;
yp=(y-ic)/py;
cpoint.setWorldCoordinates(xp*Z,yp*Z,Z);
cpoint.changeFrame(oMc);
cpointo.setWorldCoordinates(cpoint.get_X(),cpoint.get_Y(),cpoint.get_Z());
normw=normo;
norm=R*normo;
icpoint0 = icpoint;
invd0 = 1.0 / Z;
R0 = R;
Z0 = Z;
}

/*!
  Compute the interaction matrix and the residu vector for the face.
  The method assumes that these two objects are properly sized in order to be
  able to improve the speed with the use of SubCoVector and subMatrix.

  \warning The function preCompute must be called before the this method.

  \param _R : the residu vector
  \param _J : the interaction matrix
*/
void
apKltControlPoint::computeInteractionMatrixErrorH(const vpImage<unsigned char> &_I)
{
    unsigned int index = 0;

    int j_cur = icpoint_curr.get_j();
    int i_cur = icpoint_curr.get_i();

    double x_cur(0), y_cur(0);
    vpPixelMeterConversion::convertPoint(cam, j_cur, i_cur, x_cur, y_cur);

    vpImagePoint iP0 = icpoint0;
    double x0(0), y0(0);
    vpPixelMeterConversion::convertPoint(cam, iP0, x0, y0);

    double x0_transform, y0_transform ;// equivalent x and y in the first image (reference)
    computeP_mu_t(x0, y0, x0_transform, y0_transform, H );

    vpDisplay::displayCross(_I,icpoint_curr,3,vpColor::green,3);
    std::cout << " H " << H << std::endl;

    vpDisplay::displayCross(_I, icpoint0, 3,vpColor::blue,3);

    vpImagePoint iP0T;
    vpMeterPixelConversion::convertPoint(cam, x0_transform, y0_transform, iP0T);
    vpDisplay::displayCross(_I,iP0T,3,vpColor::red,3);

    vpPoint Pc;
    vpColVector Pc0(3);
    vpColVector ip0(2);
    vpImagePoint ip;
    ip.set_i(i_cur);
    ip.set_j(j_cur);
    double x=x_cur;
    double y=y_cur;
    //rho= x*cos(theta)+y*sin(theta);
    vpColVector normc=R0*N;
    Pc0[0]=x*Z0+normc[0]/1;
    Pc0[1]=y*Z0+normc[1]/1;
    Pc0[2]=Z0+normc[2]/1;
    Pc.projection(Pc0,ip0);
    int ic = cam.get_v0();
    int jc = cam.get_u0();
    int j1=ip0[0]*cam.get_px()+jc;
    int i1=ip0[1]*cam.get_py()+ic;
  //fline.buildFrom(rho,theta);
  //fline.display(cam,I,vpColor::yellow,1);
   vpDisplay::displayArrow(_I,i_cur,j_cur,i1,j1,vpColor::blue,4,2,1);
    //vpDisplay::displayCross(I,ip,2,vpColor::red,4);}

   //double invZ = compute_1_over_Z(x_cur, y_cur);
   double invZ = 1/Z0;

    L[0][0] = - invZ;
    L[0][1] = 0;
    L[0][2] = x_cur * invZ;
    L[0][3] = x_cur * y_cur;
    L[0][4] = -(1+x_cur*x_cur);
    L[0][5] = y_cur;

    L[1][0] = 0;
    L[1][1] = - invZ;
    L[1][2] = y_cur * invZ;
    L[1][3] = (1+y_cur*y_cur);
    L[1][4] = - y_cur * x_cur;
    L[1][5] = - x_cur;

    error[0] = (x0_transform - x_cur);
    error[1] = (y0_transform - y_cur);

    std::cout << " error " << error[0] << " " << error[1] << std::endl;
}

double
apKltControlPoint::compute_1_over_Z(const double x, const double y)
{
  double num = cRc0_0n[0] * x + cRc0_0n[1] * y + cRc0_0n[2];
  double den = -(d0 - dt);
  return num/den;
}

/*!
  Compute the new coordinates of a point given by its \f$(x,y)\f$ coordinates
  after the homography.

  \f$ P_t = {}^{c_t}H_{c_0} . P_0 \f$

  \param x_in : the x coordinates of the input point
  \param y_in : the y coordinates of the input point
  \param x_out : the x coordinates of the output point
  \param y_out : the y coordinates of the output point
  \param _cHc0 : the homography used to transfer the point
*/
inline void
apKltControlPoint::computeP_mu_t(const double x_in, const double y_in, double& x_out, double& y_out, const vpMatrix& _cHc0)
{
  double p_mu_t_2 = x_in * _cHc0[2][0] + y_in * _cHc0[2][1] + _cHc0[2][2];

  if( fabs(p_mu_t_2) < std::numeric_limits<double>::epsilon()){
    x_out = 0.0;
    y_out = 0.0;
    throw vpException(vpException::divideByZeroError, "the depth of the point is calculated to zero");
  }

  x_out = (x_in * _cHc0[0][0] + y_in * _cHc0[0][1] + _cHc0[0][2]) / p_mu_t_2;
  y_out = (x_in * _cHc0[1][0] + y_in * _cHc0[1][1] + _cHc0[1][2]) / p_mu_t_2;
}


/*!
  compute the homography using a displacement matrix.

  the homography is given by:

  \f$ {}^cH_{c_0} = {}^cR_{c_0} + \frac{{}^cT_{c_0} . {}^tN}{d_0} \f$

  Several internal variables are computed (dt, cRc0_0n)

  \param _cTc0 : the displacement matrix of the camera between the initial position of the camera and the current camera position
  \param _cHc0 : the homography of the plane
*/
void
apKltControlPoint::computeHomography(const vpHomogeneousMatrix& _cTc0, vpHomography& _cHc0)
{
  vpRotationMatrix cRc0;
  vpTranslationVector ctransc0;

  _cTc0.extract(cRc0);
  _cTc0.extract(ctransc0);

  vpMatrix cHc0(_cHc0);

  std::cout << " ctc0 " << _cTc0 << std::endl;

  //vpGEMM(cRc0, 1.0, invd0, cRc0, -1.0, _cHc0, VP_GEMM_A_T);
   vpGEMM(ctransc0, N, -invd0, cRc0, 1.0, cHc0, VP_GEMM_B_T);
  _cHc0 /= _cHc0[2][2];

  std::cout << " N " << N << std::endl;
  std::cout << " -invd0 " << -invd0 << std::endl;

  H = cHc0;

//   vpQuaternionVector NQuat(N[0], N[1], N[2], 0.0);
//   vpQuaternionVector RotQuat(cRc0);
//   vpQuaternionVector RotQuatConj(-RotQuat.x(), -RotQuat.y(), -RotQuat.z(), RotQuat.w());
//   vpQuaternionVector partial = RotQuat * NQuat;
//   vpQuaternionVector resQuat = (partial * RotQuatConj);
//
//   cRc0_0n = vpColVector(3);
//   cRc0_0n[0] = resQuat.x();
//   cRc0_0n[1] = resQuat.y();
//   cRc0_0n[2] = resQuat.z();

  cRc0_0n = cRc0*N;

//   vpPlane p(corners[0], corners[1], corners[2]);
//   vpColVector Ncur = p.getNormal();
//   Ncur.normalize();
  N_cur = cRc0_0n;
  dt = 0.0;
  for (unsigned int i = 0; i < 3; i += 1){
    dt += ctransc0[i] * (N_cur[i]);
  }
}


void
apKltControlPoint::computeInteractionMatrixError(const vpHomogeneousMatrix &cMo,const vpImage<unsigned char> &I)
{

	vpCameraParameters cam1;
    vpFeaturePoint featurepoint;
    if(cpoint.oP[0] !=0)
    {


	cam1=cam;
    cpointo.changeFrame(cMo);
    cpointo.projection();

    vpFeatureBuilder::create(featurepoint,cpointo);

    double x0 = featurepoint.get_x();
    double y0 = featurepoint.get_y();

    double mx = 1.0/cam.get_px();
    double my = 1.0/cam.get_py();
    double xc = cam.get_u0();
    double yc = cam.get_v0();

    double alpha ;
    vpMatrix H;
    H = featurepoint.interaction();

    double x,y;
    int j =0 ;
      x = icpoint_curr.get_j() ;
      y = icpoint_curr.get_i() ;

      vpImagePoint iP0T;
      vpMeterPixelConversion::convertPoint(cam, x0, y0, iP0T);
      //icpointi = iP0T;
      //vpDisplay::displayCross(I,iP0T,4,vpColor::red,4);
      //vpDisplay::displayCross(I, icpoint0, 4,vpColor::blue,4);
      //vpDisplay::displayCross(I,icpoint_curr,4,vpColor::green,4);

      x = (x-xc)*mx ;
      y = (y-yc)*my ;

      /*double invZ = 1/Z0;

       L[0][0] = - invZ;
       L[0][1] = 0;
       L[0][2] = x * invZ;
       L[0][3] = x * y;
       L[0][4] = -(1+x*x);
       L[0][5] = y;

       L[1][0] = 0;
       L[1][1] = - invZ;
       L[1][2] = y * invZ;
       L[1][3] = (1+y*y);
       L[1][4] = - y * x;
       L[1][5] = - x;*/

      double *Lx = H[0] ;
      double *Ly = H[1] ;
      // Calculate interaction matrix for a distance
      for (int k=0 ; k < 6 ; k++)
      {

    	  if(valid && vpMath::sqr(x0-x)+  vpMath::sqr(y0-y)!=0)
    	  {
        //L[0][k] = 2*Lx[k]*(x0-x) + 2*Ly[k]*(y0-y);
        //L[1][k] = 2*Lx[k]*(x0-x) + 2*Ly[k]*(y0-y);
    	L[0][k] = Lx[k];
    	L[1][k] = Ly[k];
    	  }
    	  else
    	  {
    		  L[0][k] = 0;
    		  L[1][k] = 0;
    	  }
    	  }

      if(valid)
      {
      //error[0] = vpMath::sqr(x0-x)+  vpMath::sqr(y0-y);//
      //error[1] = vpMath::sqr(x0-x)+  vpMath::sqr(y0-y);//
    error[0] = x0-x;//
    error[1] = y0-y;//
      }
      else
      {
    	  error[0] = 0;//
    	        error[1] = 0;//
      }
    }
    else
    {
       // Calculate interaction matrix for a distance
        for (int k=0 ; k < 6 ; k++)
        {
      		  L[0][k] = 0;
      		  L[1][k] = 0;
      	  }
        error[0] = 0;//
        error[1] = 0;//
    }

}


