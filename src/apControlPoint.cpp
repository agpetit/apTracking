/*
 * apControlPoint.cpp
 *
 *  Created on: March 10, 2011
 *      Author: Antoine Petit
 */

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include "apControlPoint.h"
#include <visp/vpTrackingException.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpRobust.h>

//! Normalize an angle between -Pi and Pi
static void
normalizeAngle(double &delta)
{
  while (delta > M_PI) { delta -= M_PI ; }
  while (delta < -M_PI) { delta += M_PI ; }
}


/*!
  Basic constructor that calls the constructor of the class vpMeTracker.
*/
apControlPoint::apControlPoint():apControlPointTracker()
{
  sign = 1;
  theta_1 = M_PI/2;

  icpoint=NULL;
  cpoint=NULL;
  profile=NULL;
  norm.resize(3);
  theta=0;
  profile1[0]=0;
  profile1[1]=0;
  profile1[2]=0;
  profile1[3]=1.33;
  profile1[4]=12.2;
  profile1[5]=36.5;
  profile1[6]=12.2;
  profile1[7]=1.33;
  profile1[8]=0;
  profile1[9]=0;
  profile1[10]=0;
  isSilhouette = false;
  invnormal = false;
}

/*!
  Basic destructor.
*/
apControlPoint::~apControlPoint()
{

  if (icpoint != NULL) delete icpoint ;
  if (cpoint != NULL) delete cpoint ;
  if (profile != NULL) delete profile ;

}



/*!
  Build a 3D plane thanks the 3D coordinate of the control point and the normal vector to the surface

  \param plane : The vpPlane instance used to store the computed plane equation.
*/
void
apControlPoint::buildPlane(vpPoint &pointn, vpColVector &normal,vpPlane &plane)
{
  //Equation of the plane is given by:
  double A = normal[0];
  double B = normal[1];
  double C = normal[2];
  //double D=-(A*(double)cpoint->get_oX()+B*(double)cpoint->get_oY()+C*(double)cpoint->get_oZ());
  double D=-(A*(double)pointn.get_oX()+B*pointn.get_oY()+C*(double)pointn.get_oZ());

  double normp =  sqrt(A*A+B*B+C*C) ;
  plane.setA(A/normp) ;
  plane.setB(B/normp) ;
  plane.setC(C/normp) ;
  plane.setD(D/normp) ;
}


/*!
*/
vpFeatureLine
apControlPoint::getFeatureLine()
{
	return featureline;
}


void
apControlPoint::buildFeatureLine()
{
  vpPlane plane;
  //buildPlane(plane);
  featureline.buildFrom(rho,theta,plane.getA(),plane.getB(),plane.getC(),plane.getD());
}

void
apControlPoint::buildPLine(vpHomogeneousMatrix &oMc)
{
  vpPlane plane;
  vpPlane plane1;
  vpPlane plane2;
  buildPlane(*cpoint,norm,plane);
  vpRotationMatrix R;
  oMc.extract(R);

  vpColVector norm2;
  vpColVector V(3);
  vpColVector Vo(3);
  V[0]=((cpoint->get_oX()/cpoint->get_oZ())+1)*cpoint->get_oZ()-cpoint->get_oX();
  V[1]=((cpoint->get_oY()/cpoint->get_oZ())-cos(theta)/sin(theta))*cpoint->get_oZ()-cpoint->get_oY();
  V[2]=(-plane.getD()-V[0]*plane.getA()-V[1]*plane.getB())/plane.getC()-cpoint->get_oZ();
  Vo=R*V;
  norm2=vpColVector::cross(Vo,normw);
  buildPlane(cpointo,norm2,plane2);
  buildPlane(cpointo,normw,plane1);

  line.setWorldCoordinates(plane1.getA(),plane1.getB(),plane1.getC(),plane1.getD(),plane2.getA(),plane2.getB(),plane2.getC(),plane2.getD());
}


void
apControlPoint::buildPoint(int n, int m, double &Z, double orient, vpColVector &normo, vpHomogeneousMatrix &cMo)
{
icpoint =new vpImagePoint;
cpoint = new vpPoint;
vpColVector pc;
vpColVector po;
vpRotationMatrix R;
vpHomogeneousMatrix oMc;
oMc=cMo.inverse();
//oMc.extract(R);
cMo.extract(R);
theta=orient;
double px = cam->get_px();
double py = cam->get_py();
int jc = cam->get_u0();
int ic = cam->get_v0();
icpoint->set_i(n);
icpoint->set_j(m);
double x,y;
x=(m-jc)/px;
y=(n-ic)/py;
rho=x*cos(theta)+y*sin(theta);
cpoint->setWorldCoordinates(x*Z,y*Z,Z);

//std::cout << " control point " << x*Z << " " << y*Z << " " << Z << std::endl;
cpoint->changeFrame(oMc);
cpointo.setWorldCoordinates(cpoint->get_X(),cpoint->get_Y(),cpoint->get_Z());
normw=normo;
norm=R*normo;

buildPLine(oMc);
}

void
apControlPoint::buildSilhouettePoint(int n, int m, double &Z, double orient, vpColVector &normo, vpHomogeneousMatrix &cMo)
{
icpoint =new vpImagePoint;
cpoint = new vpPoint;
vpRotationMatrix R;
vpHomogeneousMatrix oMc;
oMc=cMo.inverse();
//oMc.extract(R);
cMo.extract(R);
theta=orient;
double px = cam->get_px();
double py = cam->get_py();
int jc = cam->get_u0();
int ic = cam->get_v0();
icpoint->set_i(n);
icpoint->set_j(m);
xs=(m-jc)/px;
ys=(n-ic)/py;
Zs=Z;
nxs = cos(theta);
nys = sin(theta);
double x,y;
x=(m-jc)/px;
y=(n-ic)/py;
cpoint->setWorldCoordinates(x*Z,y*Z,Z);
cpoint->changeFrame(oMc);
cpointo.setWorldCoordinates(cpoint->get_X(),cpoint->get_Y(),cpoint->get_Z());
normw=normo;
norm=R*normo;
buildPLine(oMc);
}


void
apControlPoint::update(const vpHomogeneousMatrix &_cMo)
{
vpColVector P(3);
vpColVector p(3);
cpointo.changeFrame(_cMo);
cpointo.projection();
double px = cam->get_px() ;
double py = cam->get_py() ;
double uc = cam->get_u0() ;
double vc = cam->get_v0() ;
double u,v;
v=py*cpointo.get_y()+vc;
u=px*cpointo.get_x()+uc;
icpoint->set_uv(u,v);
}

void
apControlPoint::updateSilhouettePoint(const vpHomogeneousMatrix &_cMo)
{
    cpointo.changeFrame(_cMo);
    cpointo.projection();
    double px = cam->get_px() ;
    double py = cam->get_py() ;
    double uc = cam->get_u0() ;
    double vc = cam->get_v0() ;
    double u,v;
    v=py*cpointo.get_y()+vc;
    u=px*cpointo.get_x()+uc;
    icpoint->set_uv(u,v);
    xs=cpointo.get_x();
    ys=cpointo.get_y();
    Zs=cpointo.get_Z();

    line.changeFrame(_cMo);
    line.projection();
    vpFeatureBuilder::create(featureline,line);
    //std::cout << "theta0 " << (double)theta << std::endl;
    double theta0 = theta;
    theta = featureline.getTheta();
    //std::cout << " theta1 " << (double)theta << std::endl;
    if(abs(theta0-theta)<M_PI/2){
        nxs = cos(theta);
        nys = sin(theta);}
    else{
        nxs = -cos(theta);
        nys = -sin(theta);
    }
}

void
apControlPoint::setMovingEdge(vpMe *_me)
{
  me = _me ;
  apControlPointTracker::setMe(me);
}

void
apControlPoint::setCameraParameters(vpCameraParameters *_cam)
{
  cam = _cam ;
}


/*!
  Initialization of the tracking. The line is defined thanks to the
  coordinates of two points corresponding to the extremities and its (\f$\rho \: \theta\f$) parameters.
  
  Remeber the equation of a line : \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$

  \param I : Image in which the line appears.
  \param ip1 : Coordinates of the first point.
  \param ip2 : Coordinates of the second point.
  \param rho : The \f$\rho\f$ parameter
  \param theta : The \f$\theta\f$ parameter
*/
void
apControlPoint::initControlPoint(const vpImage<unsigned char>& I, double cvlt)
{

  try
  {

      double delta = theta;
 	  s.init((double)icpoint->get_i(), (double)icpoint->get_j(), delta, cvlt, sign);
 	  //s.track(I,me,0);
  }
  catch(...)
  {
//    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  vpCDEBUG(1) <<" end vpMeLine::initTracking()"<<std::endl ;
}

void
apControlPoint::setProfile(const vpImage<unsigned char>& Iprec)
{
    double salpha = sin(delta);
    double calpha = cos(delta);
    int n = 0 ;
    vpImagePoint ip;

    int ii,jj;

    for(int kk = -5 ; kk <= 5 ; kk++)
    {
      ii = (icpoint->get_i()+kk*salpha);
      jj = (icpoint->get_j()+kk*calpha);

  	// Display
      //if    ((selectDisplay==RANGE_RESULT)||(selectDisplay==RANGE)) {
        ip.set_i( ii );
        ip.set_j( jj );
        //vpDisplay::displayCross(I, ip, 1, vpColor::red);
      //}

      // Copy parent's convolution
      //vpPointSite pel ;
      //pel.init(ii, jj, alpha, convlt,mask_sign) ;
      //pel.setDisplay(selectDisplay) ;// Display

  	// Add site to the query list
      //list_query_pixels[n] = pel ;
      //std::cout << "autoIm02 "   << (int)ii << " "<< (int)jj << " " << n << std::endl;

      if((int)ii>0 && (int)ii<Iprec.getHeight() && (int)jj>0 && (int)jj<Iprec.getWidth())
      {
      //std::cout << "autoIm01 "   << n << std::endl;
      profile1[n]=Iprec[(int)ii][(int)jj];

      }
      else
      {
      	profile1[n]=0;

      }
      n++ ;
    }
}

void
apControlPoint::detectSilhouette(const vpImage<unsigned char>& I, const vpImage<unsigned char>& I0, double cvlt)
{

  try
  {
	  double delta = theta;
	  vpPointSite s0;
	  vpImagePoint pbg;
	  int k=0;
	  int k1=0;
	  vpPointSite  *list_query_pixels;
	  vpPointSite sq;
	  double angFB = theta;
	  int n0;
	  s0.init((double)icpoint->get_i(), (double)icpoint->get_j(), delta, cvlt, sign);
	  list_query_pixels = s0.getQueryList(I, 4);
	  for (int n=0; n<9; n++)
	  {
		  sq = list_query_pixels[n];

		  if(I[sq.i][sq.j]==255)
		  {
			 pbg.set_i(sq.i);
			 pbg.set_j(sq.j);
			 if(n > 4)
			 {
              //n0 = n;
				 k1++;
			 }
			 k++;
		  }
	  }
	  if(k>2)
	  {
		  isSilhouette = true;
			 //if(n0>4)
		  //std::cout << " k1 " << k1 << std::endl;
		  if(k1>1)
			 {
          nxs = -nxs;
          nys = -nys;
          invnormal = true;
			 }
	  }


	  /*for (int n=0; n<9; n++)
	  {
		  sq = list_query_pixels[n];
		  if(isSilhouette)
		  {
			  vpDisplay::displayCross(I0,*icpoint,2,vpColor::green,2);
			 if(n > 4)
			 {
				 if (invnormal)
	  vpDisplay::displayCross(I0,vpImagePoint(sq.i,sq.j),2,vpColor::blue,2);
				 else vpDisplay::displayCross(I0,vpImagePoint(sq.i,sq.j),2,vpColor::red,2);

			 }
			 else
			 {
				 if (invnormal)
	  vpDisplay::displayCross(I0,vpImagePoint(sq.i,sq.j),2,vpColor::red,2);
				 else vpDisplay::displayCross(I0,vpImagePoint(sq.i,sq.j),2,vpColor::blue,2);
			 }
	  }
	  }*/

	  delete [] list_query_pixels;


  }
  catch(...)
  {
//    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  vpCDEBUG(1) <<" end vpMeLine::initTracking()"<<std::endl ;
}


void
apControlPoint::display(const vpImage<unsigned char>&I, const vpColor col, const unsigned int thickness)
{

vpDisplay::displayCross(I,*icpoint,2,col,thickness);

}

void
apControlPoint::display(const vpImage<vpRGBa>&I, const vpColor col, const unsigned int thickness)
{

vpDisplay::displayCross(I,*icpoint,2,col,thickness);

}



/*!
  Construct a list of vpMeSite moving edges at a particular sampling
  step between the two extremities of the line.

  \param I : Image in which the line appears.
*/

void
apControlPoint::initInteractionMatrixError()
{

    L.resize(6) ;
    L1.resize(6) ;
    L2.resize(6) ;
    error = 0;
    nbFeature = 1;
}


void
apControlPoint::initInteractionMatrixError30()
{

    LD.resize(11,6) ;
    errorD.resize(11) ;
    nbFeature = 11 ;
    //std::cout<<" theta 1 "<<(180/M_PI)*featureline.getTheta()<<std::endl;
}


/*!
  Compute the interaction matrix and the error vector corresponding to the line.
*/
void
apControlPoint::computeInteractionMatrixError(const vpHomogeneousMatrix &cMo,const vpImage<unsigned char> &I)
{


	vpCameraParameters cam1;
	cam1=*cam;

    line.changeFrame(cMo);
    double a,b,c,d;
    a = line.cP[4]*line.cP[3] - line.cP[0]*line.cP[7];
    b = line.cP[5]*line.cP[3] - line.cP[1]*line.cP[7];
    c = line.cP[6]*line.cP[3] - line.cP[2]*line.cP[7];
    d = a*a + b*b;
    if (d>1e-8)
    {
    line.projection();
    vpFeatureBuilder::create(featureline,line);

    double rho0 = featureline.getRho() ;
    double theta0 = featureline.getTheta();

    double co = cos(theta0);
    double si = sin(theta0);
    //std::cout<<" theta "<<(180/M_PI)*featureline.getTheta()<<std::endl;

    double mx = 1.0/cam->get_px() ;
    double my = 1.0/cam->get_py() ;
    double xc = cam->get_u0() ;
    double yc = cam->get_v0() ;

    double alpha ;
    vpMatrix H;
    H = featureline.interaction();
    //std::cout<<H<<std::endl;
    double x,y;
      x = (double)s.j ;
      y = (double)s.i ;
      //std::cout<<" x "<< x <<" y "<< y << std::endl;

      x = (x-xc)*mx ;
      y = (y-yc)*my ;

      alpha = x*si - y*co;

      double *Lrho = H[0] ;
      double *Ltheta = H[1] ;
      // Calculate interaction matrix for a distance
      for (int k=0 ; k < 6 ; k++)
      {
        L[k] = (Lrho[k] + alpha*Ltheta[k]);
      }
      error = rho0 - ( x*co + y*si) ;
    }
}

void
apControlPoint::computeInteractionMatrixErrorMH(const vpHomogeneousMatrix &cMo,const vpImage<unsigned char> &I)
{
    line.vpLine::changeFrame(cMo);

    double a,b,c,d;
    a = line.cP[4]*line.cP[3] - line.cP[0]*line.cP[7];
    b = line.cP[5]*line.cP[3] - line.cP[1]*line.cP[7];
    c = line.cP[6]*line.cP[3] - line.cP[2]*line.cP[7];
    d = a*a + b*b;
    bool valid = false;
    if (d>1e-5)
    {
    line.vpLine::projection();

    vpFeatureBuilder::create(featureline,line);
    const double rho0 = featureline.getRho() ;
    const double theta0 = featureline.getTheta();

    const double co = cos(theta0);
    const double si = sin(theta0);

    const double mx = 1.0/cam->get_px() ;
    const double my = 1.0/cam->get_py() ;
    const double xc = cam->get_u0() ;
    const double yc = cam->get_v0() ;
    const vpMatrix &H = featureline.interaction();
    double xmin, ymin;
    double errormin = 2.0;

        const int n_hyp = s.getNumCandidates();
            xmin = (s.j - xc) * mx;
            ymin = (s.i - yc) * my;
        for (int l = 0; l < n_hyp ; l++) //for each candidate of P
        {
            const vpPointSite &Pk = candidates[l+1];

            if((Pk.suppress == 0) )
            {
                const double x = (Pk.j - xc) * mx ;
                const double y = (Pk.i - yc) * my ;
                const double err = fabs(rho0 - (x * co + y * si));
                if (err <= errormin)
                {
                    errormin = err;
                    xmin = x;
                    ymin = y;
                    valid = true;

                }
            }
        }
    if (valid)
    {
    error = rho0 - (xmin * co + ymin * si);
    const double alpha = xmin * si - ymin * co;

    const double *Lrho = H[0];
    const double *Ltheta = H[1];
    // Calculate interaction matrix for a distance
    for (int k = 0 ; k < 6 ; k++)
    {
        L[k] = (Lrho[k] + alpha * Ltheta[k]);
    }
    }

    }
}


void
apControlPoint::computeInteractionMatrixError2(const vpHomogeneousMatrix &cMo,const vpImage<unsigned char> &I)
{

    cpointo.changeFrame(cMo);
    cpointo.projection();

    vpFeatureBuilder::create(featurepoint,cpointo);

    double x0 = featurepoint.get_x();
    double y0 = featurepoint.get_y();

    double mx = 1.0/cam->get_px();
    double my = 1.0/cam->get_py();
    double xc = cam->get_u0();
    double yc = cam->get_v0();

    double alpha ;
    vpMatrix H;
    H = featurepoint.interaction();
    //std::cout<<H<<std::endl;

    double x,y;
      x = (double)s.j ;
      y = (double)s.i ;
      //std::cout<<" x "<< x <<" y "<< y << std::endl;

      x = (x-xc)*mx ;
      y = (y-yc)*my ;

      double *Lx = H[0] ;
      double *Ly = H[1] ;
      // Calculate interaction matrix for a distance
      for (int k=0 ; k < 6 ; k++)
      {
    	  if(vpMath::sqr(x0-x)+  vpMath::sqr(y0-y)!=0)
        L[k] = ((x0-x)*Lx[k] + (y0-y)*Ly[k])/(sqrt(vpMath::sqr(x0-x)+vpMath::sqr(y0-y)));
    	  else
    		  L[k] =0;
      }
      error = sqrt(vpMath::sqr(x0-x)+  vpMath::sqr(y0-y));//vpMath::sqr(x0-x)+  vpMath::sqr(y0-y);//

}

void
apControlPoint::computeInteractionMatrixError3(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I, const vpImage<double> &Igrad, const vpImage<double> &Igradx, const vpImage<double> &Igrady)
{
    line.changeFrame(cMo);
    line.projection();
    cpointo.changeFrame(cMo);
    cpointo.projection();

    vpFeatureBuilder::create(featurepoint,cpointo);

    vpFeatureBuilder::create(featureline,line);


    double rho0 = featureline.getRho() ;
    double theta0 = featureline.getTheta();
    //theta0 = theta0 + M_PI/2;
    double co = cos(theta0);
    double si = sin(theta0);


    double autoProf=40.416;

    double mx = 1.0/cam->get_px();
    double my = 1.0/cam->get_py();
    double xc = cam->get_u0();
    double yc = cam->get_v0();


    double x0 = featurepoint.get_x();
    double y0 = featurepoint.get_y();
    double i0 = (double)icpoint->get_i();
    double j0 = (double)icpoint->get_j();


    double * gradN;
    gradN = new double[11];
    //grad = getProfile(Igrad);

    double * gradx;
    gradx = new double[11];
    //gradx = getProfile(Igradx);

    double * grady;
    grady = new double[11];
    //grady = getProfile(Igrady);

    int n;
      double ii , jj ;
      /*gradN =  NULL ;
      gradx = NULL;
      grady = NULL;*/

      double salpha = sin(theta0);
      double calpha = cos(theta0);
      //std::cout<<"alpha "<<(180/M_PI)*alpha<<std::endl;
      n = 0 ;
      vpImagePoint ip;

      //std::cout << "autoIm00 "   << std::endl;


      for(int kk = -5 ; kk <= 5 ; kk++)
      {
        ii = (i0+kk*salpha);
        jj = (j0+kk*calpha);

    	// Display
        //if    ((selectDisplay==RANGE_RESULT)||(selectDisplay==RANGE)) {
          ip.set_i( ii );
          ip.set_j( jj );
          vpDisplay::displayCross(I, ip, 1, vpColor::red);
        //}

        // Copy parent's convolution
        //vpPointSite pel ;
        //pel.init(ii, jj, alpha, convlt,mask_sign) ;
        //pel.setDisplay(selectDisplay) ;// Display

    	// Add site to the query list
        //list_query_pixels[n] = pel ;
        //std::cout << "autoIm02 "   << (int)ii << " "<< (int)jj << " " << n << std::endl;
        if((int)ii>0 && (int)ii<480 && (int)jj>0 && (int)jj<640)
        {
        //std::cout << "autoIm01 "   << n << std::endl;
        gradN[n]=Igrad[(int)ii][(int)jj];
        //std::cout << "autoIm02 "   << gradN[n] << std::endl;
        gradx[n]=Igradx[(int)ii][(int)jj];
        //std::cout << "autoIm03 "   << n << std::endl;
        grady[n]=Igrady[(int)ii][(int)jj];
        }
        else
        {
        	gradN[n]=0;
        	gradx[n]=0;
        	grady[n]=0;
        }
        n++ ;
      }
      //std::cout << "ok ok" << gradN[0] << " "<<  gradN[1] << " "<< gradN[2] << " "<< gradN[4] << " "<< gradN[5] << " "<< gradN[6] << " "<<  gradN[7] << " "<< gradN[8] << " "<< gradN[10] << " "<< gradN[11] << " "<<gradN[12] << " "<<  gradN[13] << " "<< gradN[14] << " "<< gradN[15] << " "<< gradN[16] << " "<<std::endl;
          //getchar();

    double alpha ;
    vpMatrix H;
    vpMatrix Hp;
    //featureline.print();
    //featureline.display(cam1,I,vpColor::red,1);
    H = featureline.interaction();
    Hp = featurepoint.interaction();
    //std::cout<<H<<std::endl;

    double x,y;
      //x = (double)list.value().j ;
      //y = (double)list.value().i ;
      //std::cout<<" x "<< x <<" y "<< y << std::endl;

      /*x = (x-xc)*mx ;
      y = (y-yc)*my ;

      alpha = x*si - y*co;*/

    double autoIm =0;


    for (int l = 0 ; l<11;l++)
    	  {

    		  autoIm += gradN[l]*gradN[l];
    	  }

    double norm2 = autoProf*sqrt(autoIm);
    //std::cout << "norm2 "   << norm2 << std::endl;
    double normCrossCorel=0;
    if(norm2>0)
    {
	  for (int l = 0 ; l<11;l++)
	  {

		  //normCrossCorel += profile1[l]*gradN[l]/norm2;
		  normCrossCorel += (profile1[l]-gradN[l])*(profile1[l]-gradN[l]);
	  }
    }
    else {
    	normCrossCorel =0;

    }

    //if (normCrossCorel>0.9)
    //std::cout << "normCrossCorr "   << normCrossCorel << std::endl;

      error = normCrossCorel;
      vpMatrix L0(11,6);
      L1.resize(6);
      L2.resize(6);

      double *Lrho = H[0];
      double *Ltheta = H[1];
      double *Lx = Hp[0];
      double *Ly = Hp[1];
      // Calculate interaction matrix for a distance
      for (int k=0 ; k < 6 ; k++)
      {
    	  for (int l = 0 ; l<11;l++)
    	  {

    		  //L0[l][k] = -(gradx[l]*(Lx[k]-(l-5)*si*Ltheta[k]) + grady[l]*(Ly[k] + (l-5)*co*Ltheta[k]));
    		  L0[l][k] = (gradx[l]*(Lx[k]-(l-5)*si*Ltheta[k]) + grady[l]*(Ly[k] + (l-5)*co*Ltheta[k]));
    		  //L0[l][k] = (1*co*(Lx[k]-(l-5)*si*Ltheta[k]) + 1*co*(Ly[k] + (l-5)*si*Ltheta[k]));

    	  }
      }
double beta;
      if(autoIm>0){
       beta = (-error+1)/autoIm;
      }
      else{
       beta = 0;
      }

      double l1 = 0;
      double l2 = 0;

      for (int k=0 ; k < 6 ; k++)
      {
    	  for (int l = 0 ; l<11;l++)
    	  {

    		  /*l1 += gradN[l]*L0[l][k];
    		  l2 += profile1[l]*L0[l][k];*/

    		  l1 += -2*(profile1[l]-gradN[l])*L0[l][k];

    	  }
    	  L1[k] = l1;
    	  L2[k] = l2;
      }


      if(norm2>0)
      {
      for (int k=0 ; k < 6 ; k++)
      {

    		 // L[j][k] = beta*L1[j][k] - L2[j][k]/norm2;
    	  L[k] = L1[k];
      }
      }
      else
      {
          for (int k=0 ; k < 6 ; k++)
          {

        		  L[k] = 0;
          }
      }

    delete []gradN;
    delete []gradx;
    delete []grady;
}


void
apControlPoint::computeInteractionMatrixError30(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I, const vpImage<double> &Igrad, const vpImage<double> &Igradx, const vpImage<double> &Igrady)
{

    line.changeFrame(cMo);

    double a,b,c,d;
    a = line.cP[4]*line.cP[3] - line.cP[0]*line.cP[7];
    b = line.cP[5]*line.cP[3] - line.cP[1]*line.cP[7];
    c = line.cP[6]*line.cP[3] - line.cP[2]*line.cP[7];
    d = a*a + b*b;
    if (d>1e-8)
    {
    line.projection();
    cpointo.changeFrame(cMo);
    cpointo.projection();

    vpFeatureBuilder::create(featurepoint,cpointo);

    vpFeatureBuilder::create(featureline,line);


    double rho0 = featureline.getRho() ;
    double theta0 = featureline.getTheta();
    //theta0 = theta0 + M_PI/2;
    double co = cos(theta0);
    double si = sin(theta0);


    double autoProf=40.416;
    //std::cout<<" theta "<<(180/M_PI)*featureline.getTheta()<<std::endl;

    double mx = 1.0/cam->get_px();
    double my = 1.0/cam->get_py();
    double xc = cam->get_u0();
    double yc = cam->get_v0();


    double x0 = featurepoint.get_x();
    double y0 = featurepoint.get_y();
    /*double i0 = (double)icpoint->get_i();
    double j0 = (double)icpoint->get_j();*/
    double i0 = y0/my + yc;
    double j0 = x0/mx + xc;


    double * gradN;
    gradN = new double[11];
    //grad = getProfile(Igrad);

    double * gradx;
    gradx = new double[11];
    //gradx = getProfile(Igradx);

    double * grady;
    grady = new double[11];
    //grady = getProfile(Igrady);


    int n;
      double ii , jj ;
      /*gradN =  NULL ;
      gradx = NULL;
      grady = NULL;*/

      double salpha = sin(theta0);
      double calpha = cos(theta0);
      //std::cout<<"alpha "<<(180/M_PI)*alpha<<std::endl;
      n = 0 ;
      vpImagePoint ip;

      //std::cout << "autoIm00 "   << std::endl;


      for(int kk = -5 ; kk <= 5 ; kk++)
      {
        ii = (i0+kk*salpha);
        jj = (j0+kk*calpha);

    	// Display
        //if    ((selectDisplay==RANGE_RESULT)||(selectDisplay==RANGE)) {
          ip.set_i( ii );
          ip.set_j( jj );
          //vpDisplay::displayCross(I, ip, 3, vpColor::red,3);
        //}


        // Copy parent's convolution
        //vpPointSite pel ;
        //pel.init(ii, jj, alpha, convlt,mask_sign) ;
        //pel.setDisplay(selectDisplay) ;// Display

    	// Add site to the query list
        //list_query_pixels[n] = pel ;
        //std::cout << "autoIm02 "   << (int)ii << " "<< (int)jj << " " << n << std::endl;
        if((int)ii>0 && (int)ii<I.getHeight() && (int)jj>0 && (int)jj<I.getWidth())
        {
        //std::cout << "autoIm01 "   << n << std::endl;
        gradN[n]=I[(int)ii][(int)jj];
        //std::cout << "autoIm02 "   << gradN[n] << std::endl;
        gradx[n]=Igradx[(int)ii][(int)jj];
        //std::cout << "autoIm03 "   << n << std::endl;
        grady[n]=Igrady[(int)ii][(int)jj];
        }
        else
        {
        	gradN[n]=0;
        }
        n++ ;
      }

      //std::cout << "ok ok" << gradN[0] << " "<<  gradN[1] << " "<< gradN[2] << " "<< gradN[4] << " "<< gradN[5] << " "<< gradN[6] << " "<<  gradN[7] << " "<< gradN[8] << " "<< gradN[10] << " "<< gradN[11] << " "<<gradN[12] << " "<<  gradN[13] << " "<< gradN[14] << " "<< gradN[15] << " "<< gradN[16] << " "<<std::endl;
          //getchar();

      //std::cout << "autoIm00 "   << std::endl;

    double alpha ;
    vpMatrix H;
    vpMatrix Hp;
    /*fline.setRhoTheta(rho,theta);
    fline.setABCD(0.3051272514,0.03509317454,-0.9516647674,9.166432506);*/
    //featureline.print();
    //featureline.display(cam1,I,vpColor::red,1);
    H = featureline.interaction();
    Hp = featurepoint.interaction();
    //std::cout<<H<<std::endl;

    double x,y;
    //int j =0 ;
      //x = (double)list.value().j ;
      //y = (double)list.value().i ;
      //std::cout<<" x "<< x <<" y "<< y << std::endl;

      /*x = (x-xc)*mx ;
      y = (y-yc)*my ;

      alpha = x*si - y*co;*/

    double autoIm =0;


    for (int l = 0 ; l<11;l++)
    	  {

    		  autoIm += gradN[l]*gradN[l];
    	  }

    double norm2 = autoProf*sqrt(autoIm);
    //std::cout << "norm2 "   << norm2 << std::endl;
    double normCrossCorel=0;
    if(norm2>0)
    {
	  for (int l = 0 ; l<11;l++)
	  {

		  //normCrossCorel += profile1[l]*gradN[l]/norm2;
		  //normCrossCorel += (profile1[l]-gradN[l])*(profile1[l]-gradN[l]);
		  errorD[l] = (profile1[l]-gradN[l]);
		  //std::cout << " grad " << gradN[l]  << std::endl;
	  }
    }
    else {
    	normCrossCorel =0;

    }

    //if (normCrossCorel>0.9)
    //std::cout << "normCrossCorr "   << normCrossCorel << std::endl;

      //error[j] = normCrossCorel;
      vpMatrix L0(11,6);
      L1.resize(6);
      L2.resize(6);

      double *Lrho = H[0];
      double *Ltheta = H[1];
      double *Lx = Hp[0];
      double *Ly = Hp[1];
      // Calculate interaction matrix for a distance
      for (int k=0 ; k < 6 ; k++)
      {
    	  for (int l = 0 ; l<11;l++)
    	  {

    		  //L0[l][k] = -(gradx[l]*(Lx[k]-(l-5)*si*Ltheta[k]) + grady[l]*(Ly[k] + (l-5)*co*Ltheta[k]));
    		  //L0[l][k] = (gradx[l]*(Lx[k]-(l-5)*si*Ltheta[k]) + grady[l]*(Ly[k] + (l-5)*co*Ltheta[k]));
    		  //L0[l][k] = (1*co*(Lx[k]-(l-5)*si*Ltheta[k]) + 1*co*(Ly[k] + (l-5)*si*Ltheta[k]));
    		  LD[l][k] = (gradx[l]*(Lx[k]-(l-5)*si*Ltheta[k]) + grady[l]*(Ly[k] + (l-5)*co*Ltheta[k]));

    	  }
      }
double beta;
/*
      if(autoIm>0){
       beta = (-error[j]+1)/autoIm;
      }
      else{
       beta = 0;
      }

      double l1 = 0;
      double l2 = 0;

      for (int k=0 ; k < 6 ; k++)
      {
    	  for (int l = 0 ; l<11;l++)
    	  {

    		  //l1 += gradN[l]*L0[l][k];
    		  //l2 += profile1[l]*L0[l][k];

    		  l1 += -2*(profile1[l]-gradN[l])*L0[l][k];

    	  }
    	  L1[j][k] = l1;
    	  L2[j][k] = l2;
      }


      if(norm2>0)
      {
      for (int k=0 ; k < 6 ; k++)
      {

    		 // L[j][k] = beta*L1[j][k] - L2[j][k]/norm2;
    	  L[j][k] = L1[j][k];
      }
      }
      else
      {
          for (int k=0 ; k < 6 ; k++)
          {

        		  L[j][k] = 0;
          }
      }*/


    delete [] gradN;
    delete [] gradx;
    delete [] grady;

    }
}

void
apControlPoint::computeInteractionMatrixError4(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I, const vpImage<double> &Igrad, const vpImage<double> &Igradx, const vpImage<double> &Igrady)
{

    line.changeFrame(cMo);
    line.projection();
    cpointo.changeFrame(cMo);
    cpointo.projection();

    vpFeatureBuilder::create(featurepoint,cpointo);
    vpFeatureBuilder::create(featureline,line);


    double rho0 = featureline.getRho() ;
    double theta0 = featureline.getTheta();
    double co = cos(theta0);
    double si = sin(theta0);


    double autoProf=40.416;
    //std::cout<<" theta "<<(180/M_PI)*featureline.getTheta()<<std::endl;

    double mx = 1.0/cam->get_px();
    double my = 1.0/cam->get_py();
    double xc = cam->get_u0();
    double yc = cam->get_v0();


    double x0 = featurepoint.get_x();
    double y0 = featurepoint.get_y();
    double i0 = (double)icpoint->get_i();
    double j0 = (double)icpoint->get_j();


    double * gradN;
    gradN = new double[11];
    //grad = getProfile(Igrad);

    double * gradx;
    gradx = new double[11];
    //gradx = getProfile(Igradx);

    double * grady;
    grady = new double[11];
    //grady = getProfile(Igrady);

    int n;
      double ii , jj ;
      /*gradN =  NULL ;
      gradx = NULL;
      grady = NULL;*/

      double salpha = sin(theta0);
      double calpha = cos(theta0);
      //std::cout<<"alpha "<<(180/M_PI)*alpha<<std::endl;
      n = 0 ;
      vpImagePoint ip;

      //std::cout << "autoIm00 "   << std::endl;


      for(int kk = -5 ; kk <= 5 ; kk++)
      {
        ii = (i0+kk*salpha);
        jj = (j0+kk*calpha);

    	// Display
        //if    ((selectDisplay==RANGE_RESULT)||(selectDisplay==RANGE)) {
          ip.set_i( ii );
          ip.set_j( jj );
          vpDisplay::displayCross(I, ip, 1, vpColor::red) ;
        //}

        // Copy parent's convolution
        //vpPointSite pel ;
        //pel.init(ii, jj, alpha, convlt,mask_sign) ;
        //pel.setDisplay(selectDisplay) ;// Display

    	// Add site to the query list
        //list_query_pixels[n] = pel ;
        //std::cout << "autoIm02 "   << (int)ii << " "<< (int)jj << " " << n << std::endl;
        if((int)ii>0 && (int)ii<480 && (int)jj>0 && (int)jj<640)
        {
        //std::cout << "autoIm01 "   << n << std::endl;
        gradN[n]=Igrad[(int)ii][(int)jj];
        //std::cout << "autoIm02 "   << gradN[n] << std::endl;
        gradx[n]=Igradx[(int)ii][(int)jj];
        //std::cout << "autoIm03 "   << n << std::endl;
        grady[n]=Igrady[(int)ii][(int)jj];
        }
        else
        {
        	gradN[n]=0;
        	gradx[n]=0;
        	grady[n]=0;
        }
        n++ ;
      }
      //std::cout << "ok ok" << gradN[0] << " "<<  gradN[1] << " "<< gradN[2] << " "<< gradN[4] << " "<< gradN[5] << " "<< gradN[6] << " "<<  gradN[7] << " "<< gradN[8] << " "<< gradN[10] << " "<< gradN[11] << " "<<gradN[12] << " "<<  gradN[13] << " "<< gradN[14] << " "<< gradN[15] << " "<< gradN[16] << " "<<std::endl;
          //getchar();

      //std::cout << "autoIm00 "   << std::endl;

    double alpha ;
    vpMatrix H;
    vpMatrix Hp;
    /*fline.setRhoTheta(rho,theta);
    fline.setABCD(0.3051272514,0.03509317454,-0.9516647674,9.166432506);*/
    //featureline.print();
    //featureline.display(cam1,I,vpColor::red,1);
    H = featureline.interaction();
    Hp = featurepoint.interaction();
    //std::cout<<H<<std::endl;

    double x,y;
      //x = (double)list.value().j ;
      //y = (double)list.value().i ;
      //std::cout<<" x "<< x <<" y "<< y << std::endl;

      /*x = (x-xc)*mx ;
      y = (y-yc)*my ;

      alpha = x*si - y*co;*/

    double autoIm = 0;


    /*for (int l = 0 ; l<11;l++)
    	  {

    		  autoIm += gradN[l]*gradN[l];
    	  }*/

    //double norm2 = autoProf*sqrt(autoIm);
    //std::cout << "norm2 "   << norm2 << std::endl;
    double normCrossCorel=0;

	  for (int l = 0 ; l<11;l++)
	  {

		  normCrossCorel += profile1[l]*gradN[l];
	  }


    //if (normCrossCorel>0.9)
    //std::cout << "normCrossCorr "   << normCrossCorel << std::endl;

      error = -normCrossCorel;
      vpMatrix L0(11,6);
      L1.resize(6);
      L2.resize(6);

      double *Lrho = H[0] ;
      double *Ltheta = H[1] ;
      double *Lx = Hp[0];
      double *Ly = Hp[1];
      // Calculate interaction matrix for a distance
      for (int k=0 ; k < 6 ; k++)
      {
    	  for (int l = 0 ; l<11;l++)
    	  {

    		  L0[l][k] = -(si*(Lx[k]-(l-5)*mx*si*Ltheta[k]) + co*(Ly[k] + (l-5)*my*co*Ltheta[k]));
    	  }
      }

      double l1 = 0;
      double l2 = 0;

      for (int k=0 ; k < 6 ; k++)
      {
    	  for (int l = 0 ; l<11;l++)
    	  {

    		  l1 += gradN[l]*L0[l][k];
    		  l2 += profile1[l]*L0[l][k];
    	  }
    	  L1[k] = l1;
    	  L2[k] = l2;
      }


      for (int k=0 ; k < 6 ; k++)
      {

    		  L[k] = - L2[k];
      }

    delete []gradN;
    delete []gradx;
    delete []grady;
}

void
apControlPoint::track(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec)
{
  //  2. On appelle ce qui n'est pas specifique
  try
  {
    apControlPointTracker::track(I,Iprec);
  }
  catch(...)
  {
    throw ;
  }

}

 void
 apControlPoint::trackMH(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec)
 {
   //  2. On appelle ce qui n'est pas specifique
   try
   {
     apControlPointTracker::trackMH(I,Iprec);

   }
   catch(...)
   {
     throw ;
   }

 }

 void
 apControlPoint::trackPred(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec)
 {
   //  2. On appelle ce qui n'est pas specifique
   try
   {
     apControlPointTracker::trackPred(I,Iprec);
   }
   catch(...)
   {
     throw ;
   }
 }

#endif

