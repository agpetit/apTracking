/*
 * apMHMeLine.cpp
 *
 *  Created on: Jul 6, 2012
 *      Author: agpetit
 */

#include <visp/vpTrackingException.h>
#include <visp/vpRobust.h>
#include <visp/vpFeatureDisplay.h>

#include "apMHMeLine.h"

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
apMHMeLine::apMHMeLine():apMHMeTracker()
{
  sign = 1;
  theta_1 = M_PI/2;
}

/*!
  Basic destructor.
*/
apMHMeLine::~apMHMeLine()
{
  list.resize(0);
}

apMHMeLine&
apMHMeLine::operator = (apMHMeLine& p)
{

  list = p.list;
  me = p.me;
  selectDisplay = p.selectDisplay ;
  return *this;
}


double apMHMeLine::leastSquareLine(const vpImage<unsigned char> &I, std::vector<apMHMeSite> &tab,const vpCameraParameters &Cam,
					apMeLine &line,
					double &rho_m, double &theta_m)
{
// 	cout << "start least square" << endl;
// 	double t0 = vpTime::measureTimeMs();
	// Construction du systeme Ax=B
	try{
// 		cout << "size : " << tab.size() << endl;
		int tab_size = tab.size();
		vpMatrix A(tab_size,2) ;
		vpColVector x(2),x_1(2) ;
		x_1 = 0;
		vpColVector B(tab_size);
		vpMatrix D(tab_size,tab_size) ;
		D.setIdentity() ;
		vpMatrix DA, DAmemory ;
		vpColVector DAx ;
		vpColVector w(tab_size) ;
		w = 1;
		int iter = 0;
		double distance = 100;
		vpRobust r(tab_size);
// 		r.setThreshold(2);
		r.setIteration(0) ;
		vpColVector residu(tab_size);
		double error = 0;

		apMHMeSite P;
		if (fabs(line.b)>0.9)
		{
		// a i + j + c = 0
		// A = (i 1)   B = (-j)
			for (int i = 0; i < tab_size; i++)
			{
				P = tab[i];
				A[i][0] = P.ifloat;
				A[i][1] = 1;
				B[i] = -P.jfloat;
			}

			while (iter < 5 && distance > 0.05)
			{
				DA = D*A ;
				try {
// 					cout << "Try compute pseudo-inverse..." << endl;
					x = DA.pseudoInverse(1e-26) *D*B ;
// 					cout << "Compute pseudo-inverse ok" << endl;
				}
				catch(...)
				{
					cout << "error in pseudoInverse" << endl;
				}

				residu = B - A*x;
				r.setIteration(iter) ;
				r.MEstimator(vpRobust::TUKEY,residu,w) ;

				int k = 0;
				for (int j=0 ; j < tab_size ; j++)
				{
					D[k][k] =w[k]  ;
					k++;
				}
				iter++ ;
				distance = fabs(x[0]-x_1[0])+fabs(x[1]-x_1[1]);
				x_1 = x;
			}

			// mise a jour de l'equation de la droite
			line.a = x[0] ;
			line.b = 1 ;
			line.c = x[1] ;

			double s =sqrt( vpMath::sqr(line.a)+vpMath::sqr(line.b)) ;
			line.a /= s ;
			line.b /= s ;
			line.c /= s ;
			error = residu.euclideanNorm();
		}
		else
		{
		// Construction du systeme Ax=B
		// i + bj + c = 0
		// A = (j 1)   B = (-i)
			for (int i = 0; i < tab_size; i++)
			{
				P = tab[i];
				A[i][0] = P.jfloat;
				A[i][1] = 1;
				B[i] = -P.ifloat;
			}
			while (iter < 5 && distance > 0.05)
			{
				DA = D*A ;
				try{
// 					cout << "Try compute pseudo-inverse..." << endl;
					x = DA.pseudoInverse(1e-26) *D*B ;
// 					cout << "Compute pseudo-inverse ok" << endl;
				}
				catch(...)
				{
					cout << "error in pseudoInverse" << endl;
				}
				residu = B - A*x;
				r.setIteration(iter) ;
				r.MEstimator(vpRobust::TUKEY,residu,w) ;

				int k = 0;
				for (int j=0 ; j < tab_size ; j++)
				{
					D[k][k] =w[k]  ;
					k++;
				}
				iter++ ;
				distance = fabs(x[0]-x_1[0])+fabs(x[1]-x_1[1]);
				x_1 = x;
			}

			line.a = 1 ;
			line.b = x[0] ;
			line.c = x[1] ;

			double s = sqrt(vpMath::sqr(line.a)+vpMath::sqr(line.b)) ;
			line.a /= s ;
			line.b /= s ;
			line.c /= s ;

			error = residu.euclideanNorm();
		}

		//!Compute other parameters
		apMeLine curr_line(line.a, line.b, line.c);//compute rho_i theta_i from pixel measures
		double rho_i,rho_u;
		double theta_i, theta_u;
		rho_i = curr_line.getRho();
		theta_i = curr_line.getTheta();

	// 	std::cout << "a : " << a << " , b : " << b << " , c : " << c << std::endl;
	// 	std::cout << "rho_i : " << rho_i << " , theta_i : " << theta_i << std::endl;
// 		cout << "end least square" << endl;
		//!Convert to uv coordinates:
		//Gives the rho and theta coordinates in the (u,v) coordinate system.
		theta_u = theta_i;
		rho_u = rho_i;
		if (theta_i <= 0 && theta_i > -M_PI)
			theta_i += 2*M_PI;
		if (theta_i >= 0 && theta_i < M_PI/2)
		{
			theta_u = M_PI/2 - theta_i;
		}
		else if (theta_i >= M_PI/2 && theta_i < 3*M_PI/2)
		{
			theta_u = 3*M_PI/2 + M_PI - theta_i;
		}
		else if (theta_i >= 3*M_PI/2 && theta_i <= 2*M_PI)
		{
			theta_u = M_PI/2 + 2*M_PI - theta_i;
		}
		while (theta_u > M_PI)  { theta_u -= 2*M_PI ; }
		while (theta_u < -M_PI) { theta_u += 2*M_PI ; }

	// 	std::cout << "rho_u : " << rho_u << " , theta_u : " << theta_u << std::endl;

		//!Convert to meters
		vpPixelMeterConversion::convertLine(*cam, rho_u,theta_u, rho_m, theta_m);

		line = curr_line;
		line.setTheta(theta_m);
		line.setRho(rho_m);

		return (error/(float)tab_size);
	// 	curr_line.display(I);
	// 	double t1 = vpTime::measureTimeMs();
	// 	double dt = t1-t0;
	// 	cout << "Least square time : " << dt << endl;
	}
	catch(...)
	{
		cout << "Erreur caught in least square computation"<<endl;
		return -1;
	}


}


/*!
	For multi-hypothesis case
*/
int
apMHMeLine::numberOfSignal_MH()
{
	int number_signal_MH = 0;

	// Loop through all the points tracked from the contour
	int nb_hyp = list[0].getNumCandidates();
    for (int k = 0; k<list.size(); k++)
    {
		if(list[k].suppress == 0)
		{
			for (int i = 0; i < nb_hyp; i++)
			{
				if(!((list[k].candidateList)==NULL))
				{
					if(list[k].candidateList[i].suppress == 0)
						number_signal_MH++;
				}
				else
					number_signal_MH++;
			}
		}
	}

	return number_signal_MH;
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
apMHMeLine::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho, double theta)
{
  vpCDEBUG(1) <<" begin vpMeLine::initTracking()"<<std::endl ;

  try
  {
    //  1. On fait ce qui concerne les dirties (peut etre vide)
    // Points extremites
    PExt[0].ifloat = (float)ip1.get_i();
    PExt[0].jfloat = (float)ip1.get_j();
    PExt[1].ifloat = (float)ip2.get_i();
    PExt[1].jfloat = (float)ip2.get_j();

    this->rho = rho;
    this->theta = theta;

    a = cos(theta);
    b = sin(theta);
    c = -rho;

    double d = sqrt(vpMath::sqr(ip1.get_i()-ip2.get_i())+vpMath::sqr(ip1.get_j()-ip2.get_j())) ;

    expecteddensity = d / (double)me->sample_step;

    delta = - theta + M_PI/2.0;
    normalizeAngle(delta);
    delta_1 = delta;

    sample(I);
   //std::cout << " track " << std::endl;
    //apMHMeTracker::track(I);
  }
  catch(...)
  {
//    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  vpCDEBUG(1) <<" end vpMeLine::initTracking()"<<std::endl ;
}


void
apMHMeLine::initTrackingG(const vpImage<unsigned char> &I, const vpImage<unsigned char> &gradMap, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho, double theta)
{
  vpCDEBUG(1) <<" begin vpMeLine::initTracking()"<<std::endl ;

  try
  {
    //  1. On fait ce qui concerne les dirties (peut etre vide)
    // Points extremites
    PExt[0].ifloat = (float)ip1.get_i();
    PExt[0].jfloat = (float)ip1.get_j();
    PExt[1].ifloat = (float)ip2.get_i();
    PExt[1].jfloat = (float)ip2.get_j();

    this->rho = rho;
    this->theta = theta;

    a = cos(theta);
    b = sin(theta);
    c = -rho;

    double d = sqrt(vpMath::sqr(ip1.get_i()-ip2.get_i())+vpMath::sqr(ip1.get_j()-ip2.get_j())) ;

    expecteddensity = d / (double)me->sample_step;

    delta = - theta + M_PI/2.0;
    normalizeAngle(delta);
    delta_1 = delta;

    sample(I);
   //std::cout << " track " << std::endl;
    apMHMeTracker::trackG(I,gradMap);
  }
  catch(...)
  {
//    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  vpCDEBUG(1) <<" end vpMeLine::initTracking()"<<std::endl ;
}

void
apMHMeLine::initTrackingP(const vpImage<unsigned char> &I, std::vector<apControlPoint*> &points)
{
  vpCDEBUG(1) <<" begin vpMeLine::initTracking()"<<std::endl ;

  try
  {
	  // Delete old list
	  list.resize(0);
      apControlPoint *p;
	  vpImagePoint ip;
	  for(int i=0; i<points.size(); i++)
	  {
		  p = points[i];
	      apMHMeSite pix ; //= list.value();
	      pix.init(p->icpoint->get_i(), p->icpoint->get_j(), p->get_theta(), 0, sign) ;
	      //pix.track0(I,me,0);
	      pix.setDisplay(selectDisplay) ;

	      if(vpDEBUG_ENABLE(3))
	      {
		ip.set_i(p->icpoint->get_i());
		ip.set_j(p->icpoint->get_j());
		vpDisplay::displayCross(I, ip, 2, vpColor::red);
	      }
	      list.push_back(pix);

	  }

  }
  catch(...)
  {
//    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  vpCDEBUG(1) <<" end vpMeLine::initTracking()"<<std::endl ;
}


/*!
  Construct a list of vpMeSite moving edges at a particular sampling
  step between the two extremities of the line.

  \param I : Image in which the line appears.
*/
void
apMHMeLine::sample(const vpImage<unsigned char>& I)
{
  int rows = I.getHeight();
  int cols = I.getWidth();
  double n_sample;

  if (me->sample_step==0)
  {
    vpERROR_TRACE("function called with sample step = 0") ;
    throw(vpTrackingException(vpTrackingException::fatalError,
			      "sample step = 0")) ;
  }

  // i, j portions of the line_p
  double diffsi = PExt[0].ifloat-PExt[1].ifloat;
  double diffsj = PExt[0].jfloat-PExt[1].jfloat;

  double length_p = sqrt((vpMath::sqr(diffsi)+vpMath::sqr(diffsj)));

  // number of samples along line_p
  n_sample = length_p/(double)me->sample_step;

  double stepi = diffsi/(double)n_sample;
  double stepj = diffsj/(double)n_sample;

  // Choose starting point
  double is = PExt[1].ifloat;
  double js = PExt[1].jfloat;

  // Delete old list
  list.resize(0);

  //std::cout << " sample1 " << (double)n_sample << std::endl;
  // sample positions at i*me->sample_step interval along the
  // line_p, starting at PSiteExt[0]

  vpImagePoint ip;
  for(int i=0; i<=vpMath::round(n_sample); i++)
  {
    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(is), vpMath::round(js), 0, rows, cols))
    {
      apMHMeSite pix ; //= list.value();
      pix.init((int)is, (int)js, delta, 0, sign) ;

      //pix.track0(I,me,0);

      pix.setDisplay(selectDisplay) ;

      if(vpDEBUG_ENABLE(3))
      {
	ip.set_i( is );
	ip.set_j( js );
	vpDisplay::displayCross(I, ip, 2, vpColor::red);
      }

      list.push_back(pix);
    }
    is += stepi;
    js += stepj;

  }


  vpCDEBUG(1) << "end vpMeLine::sample() : ";
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl  ;
}


/*!
  Suppress the moving which belong no more to the line.

  \param I : The image.
*/
void
apMHMeLine::suppressPoints(const vpImage<unsigned char> & I)
{
  int nbrelmt;
  nbrelmt = list.size();
  for (int k = 0; k<list.size(); k++)
  {
    apMHMeSite s = list[k] ;//current reference pixel

  if (fabs(sin(theta)) > 0.9) // Vertical line management
  {
    if ((s.i < imin) ||(s.i > imax))
    {
      s.suppress = 1;
    }
  }

  else if (fabs(cos(theta)) > 0.9) // Horizontal line management
  {
    if ((s.j < jmin) || (s.j > jmax))
    {
      s.suppress = 1;
    }
  }

  else
  {
    if ((s.i < imin) ||(s.i > imax) || (s.j < jmin) || (s.j > jmax) )
    {
      s.suppress = 1;
    }
  }

  if (s.suppress != 0)
    {list.erase(list.begin() + k);
    k--;
    }

  }
  nbrelmt = list.size();

}


/*!
 Seek along the line defined by its equation, the two extremities of the line. This function is useful in case of translation of the line.

 \param I : Image in which the line appears.
*/
void
apMHMeLine::seekExtremities(const vpImage<unsigned char> &I)
{
  vpCDEBUG(1) <<"begin vpMeLine::sample() : "<<std::endl ;

  int rows = I.getHeight() ;
  int cols = I.getWidth() ;
  double n_sample;

  if (me->sample_step==0)
  {

    vpERROR_TRACE("function called with sample step = 0") ;
    throw(vpTrackingException(vpTrackingException::fatalError,"sample step = 0")) ;
  }

  // i, j portions of the line_p
  double diffsi = PExt[0].ifloat-PExt[1].ifloat;
  double diffsj = PExt[0].jfloat-PExt[1].jfloat;

  double s = vpMath::sqr(diffsi)+vpMath::sqr(diffsj) ;

  double di = diffsi/sqrt(s) ; // pas de risque de /0 car d(P1,P2) >0
  double dj = diffsj/sqrt(s) ;

  double length_p = sqrt(s); /*(vpMath::sqr(diffsi)+vpMath::sqr(diffsj))*/

  // number of samples along line_p
  n_sample = length_p/(double)me->sample_step;
  double sample = (double)me->sample_step;

  apMHMeSite P ;
  P.init((int) PExt[0].ifloat, (int)PExt[0].jfloat, delta_1, 0, sign) ;
  P.setDisplay(selectDisplay) ;

  int  memory_range = me->range ;
  me->range = 1 ;

  for (int i=0 ; i < 3 ; i++)
  {
    P.ifloat = P.ifloat + di*sample ; P.i = (int)P.ifloat ;
    P.jfloat = P.jfloat + dj*sample ; P.j = (int)P.jfloat ;


    if ((P.i < imin) ||(P.i > imax) || (P.j < jmin) || (P.j > jmax) )
    {
      if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j,5,vpColor::cyan) ;
    }
    else
    //if(!outOfImage(P.i, P.j, 5, rows, cols))
    {
      P.track(I,me,false) ;

      if (P.suppress ==0)
      {
        list.push_back(P) ;
        if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 5, vpColor::green) ;
      }
      else
        if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 10, vpColor::blue) ;
    }
  }

  P.init((int) PExt[1].ifloat, (int)PExt[1].jfloat, delta_1, 0, sign) ;
  P.setDisplay(selectDisplay) ;
  for (int i=0 ; i < 3 ; i++)
  {
    P.ifloat = P.ifloat - di*sample ; P.i = (int)P.ifloat ;
    P.jfloat = P.jfloat - dj*sample ; P.j = (int)P.jfloat ;


    if ((P.i < imin) ||(P.i > imax) || (P.j < jmin) || (P.j > jmax) )
    {
      if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j,5,vpColor::cyan) ;
    }

  else
  //if(!outOfImage(P.i, P.j, 5, rows, cols))
    {
      P.track(I,me,false) ;

      if (P.suppress ==0)
      {
				list.push_back(P) ;
				if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 5, vpColor::green) ;
      }
      else
				if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 10, vpColor::blue) ;
    }
  }

  me->range = memory_range ;

  vpCDEBUG(1) <<"end vpMeLine::sample() : " ;
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl  ;
}

/*!
  Resample the line if the number of sample is less than 50% of the
  expected value.

  \note The expected value is computed thanks to the length of the
  line and the parameter which indicates the number of pixel between
  two points (vpMe::sample_step).

  \param I : Image in which the line appears.
*/
void
apMHMeLine::reSample(const vpImage<unsigned char> &I)
{
  double d = sqrt(vpMath::sqr(PExt[0].ifloat-PExt[1].ifloat)+vpMath::sqr(PExt[0].jfloat-PExt[1].jfloat)) ;

  int n = numberOfSignal() ;
  double expecteddensity = d / (double)me->sample_step;

  if ((double)n<0.5*expecteddensity && n > 0)
  {
    double delta_new = delta;
    delta = delta_1;
    sample(I) ;
    delta = delta_new;
    //  2. On appelle ce qui n'est pas specifique
    {
      apMHMeTracker::initTracking(I) ;
    }
  }
}


/*!
  Resample the line if the number of sample is less than 50% of the
  expected value.

  \note The expected value is computed thanks to the length of the
  line and the parameter which indicates the number of pixel between
  two points (vpMe::sample_step).

  \param I : Image in which the line appears.
  \param ip1 : The first extremity of the line.
  \param ip2 : The second extremity of the line.
*/
void
apMHMeLine::reSample(const vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2)
{
  double d = sqrt(vpMath::sqr(ip1.get_i()-ip2.get_i())+vpMath::sqr(ip1.get_j()-ip2.get_j())) ;

  int n = list.size();//numberOfSignal() ;
  expecteddensity = d / (double)me->sample_step;

  if ((double)n<0.5*expecteddensity && n > 0)
  {
    double delta_new = delta;
    delta = delta_1;
    PExt[0].ifloat = (float)ip1.get_i() ;
    PExt[0].jfloat = (float)ip1.get_j() ;
    PExt[1].ifloat = (float)ip2.get_i() ;
    PExt[1].jfloat = (float)ip2.get_j() ;
    sample(I) ;
    delta = delta_new;
    apMHMeTracker::track(I) ;
  }
}

/*!
  Set the alpha value of the different vpMeSite to the value of delta.
*/
void
apMHMeLine::updateDelta()
{
  apMHMeSite p ;

  double diff = 0;

  if(fabs(theta) == M_PI )
  {
    theta = 0 ;
  }

  diff = fabs(theta - theta_1);
  if (diff > M_PI/2.0)
  sign *= -1;

  theta_1 = theta;

  delta = - theta + M_PI/2.0;
  normalizeAngle(delta);

  for (int i=0 ; i < list.size() ; i++)
  {
    p = list[i] ;
    p.alpha = delta ;
    p.mask_sign = sign;
    list[i] = p;
  }
  delta_1 = delta;
}

/*!
 Track the line in the image I.

 \param I : Image in which the line appears.
 */
void
apMHMeLine::track(const vpImage<unsigned char> &I)
{
  //  2. On appelle ce qui n'est pas specifique
  try
  {
    apMHMeTracker::track(I);


    //// 3. On revient aux droites
     //{
     //  // supression des points rejetes par les ME
     //  ////suppressPoints() ;
     //  //setExtremities() ;
     //std::cout << "After track"<<std::endl;
     //vpDisplay::flush(I);
     //vpDisplay::getClick(I);

     //  // Estimation des parametres de la droite aux moindres carre
     //  try
     //  {
   		////!TODO: find lines !!
   		//
   		//int nblines = findLinesByKmean(I,0);
   		//std::cout<< nblines <<" lines have been found"<<std::endl;
   		//if (nblines>1)
   		//	std::cout <<" -- several lines candidates !" <<std::endl;
   		//leastSquare() ;
     //  }
     //  catch(...)
     //  {
     //    vpERROR_TRACE("Error caught") ;
   	 // throw ;
     //  }


     //  // recherche de point aux extremites de la droites
     //  // dans le cas d'un glissement
     //  seekExtremities(I) ;
       //suppressPoints(I) ;
     //  setExtremities() ;
     //  //reechantillonage si necessaire
     //  reSample(I) ;
   		//

     //  // remet a jour l'angle delta pour chaque point de la liste
     //  updateDelta() ;
     //}

  }
  catch(...)
  {
    throw ;
  }

  // supression des points rejetes par les ME
  //  suppressPoints(I);
  //  setExtremities();
}

void
apMHMeLine::trackG(const vpImage<unsigned char> &I, const vpImage<unsigned char> &gradMap)
{
  //  2. On appelle ce qui n'est pas specifique
  try
  {
    apMHMeTracker::trackG(I, gradMap);
    //// 3. On revient aux droites
     //{
     //  // supression des points rejetes par les ME
     //  ////suppressPoints() ;
     //  //setExtremities() ;
      //std::cout << "After track"<<std::endl;
     //vpDisplay::flush(I);
     //vpDisplay::getClick(I);

     //  // Estimation des parametres de la droite aux moindres carre
     //  try
     //  {
   		////!TODO: find lines !!
   		//


    int nblines = findLinesByKmean_2(I,0);


   		//std::cout<< nblines <<" lines have been found"<<std::endl;
   		//if (nblines>1)
   		//	std::cout <<" -- several lines candidates !" <<std::endl;
   		//leastSquare() ;
     //  }
     //  catch(...)
     //  {
     //    vpERROR_TRACE("Error caught") ;
   	 // throw ;
     //  }


     //  // recherche de point aux extremites de la droites
     //  // dans le cas d'un glissement
     //  seekExtremities(I) ;

       //suppressPoints(I) ;

     //  setExtremities() ;
     //  //reechantillonage si necessaire
     //  reSample(I) ;
   		//

     //  // remet a jour l'angle delta pour chaque point de la liste
     //  updateDelta() ;
     //}

  }
  catch(...)
  {
    throw ;
  }

  // supression des points rejetes par les ME
  //  suppressPoints(I);
  //  setExtremities();
}


/*!
	\brief : Search for lines from vpSites, using Kmean classification approach.

 \param I : Image in which  to display
 \param display_mode : Display mode
 \return : number of found lines

*/
int apMHMeLine::findLinesByKmean(const vpImage< unsigned char> &I,int display_mode)
{
	//double t1 = vpTime::measureTimeMs();

	//if (DEBUG_LEVEL1)
		std::cout << "begin apMHMeLine::findLinesByKmean() " << std::endl ;
// 	cout << "----------------------------------------"<< endl;
	points_vect.clear();
	//lines_vect.clear();
	weights.clear();


// 	double t0 = vpTime::measureTimeMs();
	//!Init parameters
	int nbGdSites_min = 10;
	bool not_finished = true;
	unsigned int iter = 0;
	int n_hyp = (&(list[0]))->getNumCandidates();

	//std::cout << "n_hyp : " <<n_hyp<<std::endl;
//n_hyp=2;
	float error = 0;

	int nbclasses = 1;
	int nbIterMax = 10;
	float rate_nbpts_min = 0.5;

        vpColVector class_size;
	vpColVector numberOfCandidates;

	class_size.resize(nbclasses);
	vpColVector err;

	float err_min = 9999999;
	float err_max = 0;

	//!*************************************************************************
	//!--------------------------Initialisation---------------------------------
	//!*************************************************************************

	//!copy good points in a vector and get number of classes (max number of good candidates)
// 	std::cout << "initialise vectors" << std::endl;
	int nbGoodSites = 0;
	std::vector< std::vector<apMHMeSite> > site_vect;
	apMHMeSite Pk;
	apMHMeSite Pk0;
	std::vector<apMHMeSite> Pk1(n_hyp);
	apMHMeSite Pk2;

	int i1, i2 ;
	i1 = i2 = 0;
	list.front();
	for (int l =0; l<list.size(); l++)
	{
		if (list[l].suppress == 0)
		{
			nbGoodSites++;
			site_vect.resize(nbGoodSites);
			i2=0;
			for (int k = 0; k < n_hyp ; k++) //for each candidate of P
			{
				Pk = ((&(list[l]))->candidateList[k]);
				if((Pk.suppress == 0) )
				{
					site_vect[i1].push_back(Pk);
					i2++;
					if(i2>nbclasses)
					{
						nbclasses = i2;
					}
				}
			}
			i1++;
		}
	}
	int nbGoodSiteTotal = numberOfSignal_MH();
	err.resize(nbclasses);
	err = 0;

	if (nbGoodSites > nbGdSites_min)
	{
 		/*std::cout << "-- nbclasses : " << nbclasses << std::endl;
 		std::cout << "nbGoodSites : " << nbGoodSites << std::endl;
 		std::cout << "nbGoodSiteTotal : " << nbGoodSiteTotal << std::endl;
	 	std::cout << "size site_vect : " << site_vect.size() << std::endl;*/
		class_size.resize(nbclasses);
		numberOfCandidates.resize(nbGoodSites);
		numberOfCandidates = 0;

		//!Construct the class_vect tab in which the process will be made
		//!Also init class_size vect and numberOfCandidates vect in the same time
		std::vector< std::vector<apMHMeSite*> > class_vect;
		apMHMeSite* site;
		class_vect.resize(nbclasses);
		for (i2 = 0 ; i2 < nbclasses ; i2++)
			class_vect[i2].resize(nbGoodSites);

		for (i1 = 0 ; i1 < nbGoodSites ; i1++)
		{
			for (i2 = 0 ; i2 < nbclasses ; i2++)
			{
				numberOfCandidates[i1] = site_vect[i1].size();
				if (i2 < site_vect[i1].size())
				{
					/*site = new apMHMeSite;
					*site = site_vect[i1][i2];
					class_vect[i2][i1] = site;*/
					class_vect[i2][i1] = &(site_vect[i1][i2]);
					class_size[i2] ++;
				}

				else
					class_vect[i2][i1] = NULL;
			}
		}

		////!display classification
		//std::cout << "K-mean init site_vect"<<std::endl;
 		vpColor col = vpColor::black;
 	//	vpDisplay::display(I);
 	//	for(unsigned int i(0);i<nbclasses;++i){
		//
 	//		//!Display points
 	//		if (i == 0)
 	//			col = vpColor::green;
 	//		else if (i == 1)
 	//			col = vpColor::blue;
 	//		else if (i == 2)
 	//			col = vpColor::red;
		//	else if (i == 3)
 	//			col = vpColor::black;
 	//		for(i1=0;i1 < nbGoodSites; i1++)
 	//		{
 	//			if (i < site_vect[i1].size())
 	//				site_vect[i1][i].display(I,col,7);
 	//		}
 	//	}
 	//	vpDisplay::flush(I);
 	//	vpDisplay::getClick(I);
		//vpImage<vpRGBa> ItmpGet;
		//vpDisplay::getImage(I,ItmpGet);
		//vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinit.ppm");


	// 	std::cout << "class vect init ok " << std::endl;
// 		std::cout << "-- " << std::endl;
// 		for (i1 = 0 ; i1 < nbGoodSites ; i1++)
// 		{
// 			std::cout << "nOfCand "<< i1 << " : "<< numberOfCandidates[i1] << " , " ;
// 		}
// 		std::cout << std::endl << "-- " << std::endl;

		//! nbclass_corr considers only the classes with more than nbpts_min candidates.
		int nbclasses_corr = 0;
		for (int i = 0 ; i < nbclasses; i++)
		{
			if (class_size[i] > (int)(rate_nbpts_min*nbGoodSites))
			{
				nbclasses_corr ++;
			}
		}
// 		std::cout << "-- nbclasses long enough : " << nbclasses_corr << std::endl;
// 		std::cout << "-classe size : " <<endl<< class_size << std::endl;


		//!Vectors to memorise lines (result):
		std::vector<apMeLine*> mean_lines;
		mean_lines.resize(nbclasses);
		std::vector<double> mean_grad;
		mean_grad.resize(nbclasses);
		double grad;
		double mean_grad_ = 0;

		apMeLine *curr_line;
		double theta_m, rho_m;
		double co, si;
		double x,y;

		std::vector<apMHMeSite> vect_temp;

		//!init means.
		for (i2 = 0 ; i2 < nbclasses ; i2++)
		{
			//mean_grad_ = 0;
			if (class_size [i2] > (int)(rate_nbpts_min*nbGoodSites))
			{
				vect_temp.clear();
				for (i1 = 0 ; i1 < nbGoodSites ; i1++)
				{
					if (class_vect[i2][i1] != NULL)
						vect_temp.push_back(*(class_vect[i2][i1]));
					    //mean_grad_ += abs(class_vect[i2][i1]->convlt);
				}
				curr_line = new apMeLine;
				leastSquareLine(I,vect_temp, *cam, *curr_line, rho_m, theta_m);
				mean_lines[i2] = curr_line;
				//mean_grad[i2] = (mean_grad_)/nbGoodSites;
				//std::cout << " i2 "<< i2 << " mean line a " << mean_lines[i2]->a << " mean line b " << mean_lines[i2]->b << " mean line c " << mean_lines[i2]->c << std::endl;
			}
		}
	// 	double t1 = vpTime::measureTimeMs();
	// 	vpDisplay::flush(I);
	// 	vpDisplay::getClick(I);
	// 	cout << "Time init : " << t1-t0 <<endl;


		////!display classification
		//std::cout << "K-mean init "<<std::endl;
 	//	col = vpColor::black;
 	//	vpDisplay::display(I);
 	//	for(unsigned int i(0);i<nbclasses;++i){
 	//		//!Display line
 	//		col = vpColor::black;
		//	/*if (class_size [i] > (int)(rate_nbpts_min*nbGoodSites))*/
 	//			mean_lines[i].display(I,col,2);
 	//		vpDisplay::flush(I);
 	//
 	//		//!Display points
 	//		if (i == 0)
 	//			col = vpColor::green;
 	//		else if (i == 1)
 	//			col = vpColor::blue;
 	//		else if (i == 2)
 	//			col = vpColor::red;
		//	else if (i == 3)
 	//			col = vpColor::black;
 	//		for(i1=0;i1 < nbGoodSites; i1++)
 	//		{
 	//			if (class_vect[i][i1] != NULL)
 	//				class_vect[i][i1]->display(I,col,7);
 	//		}
 	//	}
 	//	vpDisplay::flush(I);
 	//	vpDisplay::getClick(I);
		//
		//vpDisplay::getImage(I,ItmpGet);
		//vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinit2.ppm");

	//!*************************************************************************
	//!-------------------------------LOOP--------------------------------------
	//!*************************************************************************
		std::vector< std::vector<apMHMeSite> >::iterator it1;
		std::vector<apMHMeSite>::iterator it2;

		std::cout  << " 0 loop mean line a " << mean_lines[0]->a << " mean line b " << mean_lines[0]->b << " mean line c " << mean_lines[0]->c << std::endl;

		int nbchange=1 ;
		bool change = false;
		int iteration=0 ;
		int classeMin = 0;
		double dmin = 300;
	// 	double dminAllCand = 300;
	// 	int i2MinAllCand;
	// 	int classMinAllCand;
		double d = 0;
		vpMatrix distances (nbclasses,nbclasses);
		vpColVector classmin(nbclasses);
		vpColVector testdiff(nbclasses);
		bool alldiff = true;
		std::vector<apMHMeSite*> vectCand;
		vectCand.resize(nbclasses);
		//apMHMeSite* site0;
		double dcand;

		double dgrad=0;
		double wgrad = 1/2;

// 		cout << "Start loop Kmean" << endl;
		while((nbchange!=0)&&(iter < nbIterMax))
		{
			iter++;
			double t2 = vpTime::measureTimeMs();
	// 		cout << "*** " <<iteration++<<" ***" << endl ;
			nbchange  = 0;

			mean_grad_ = 0;

			i1 = i2 = 0;
			//loop on the points
			for (i1 = 0 ; i1 < nbGoodSites ; i1++)
			{
	// 			double t4 = vpTime::measureTimeMs();
				i2 = 0;
				change = false;
				classmin = -1;
				testdiff = 0;
				distances = -1;
				alldiff = true;
// 				cout << "%%-- i1 : " <<i1<<" ----" << endl ;

				//!For each candidate, compute its distance to each valid line and make class permutations
				for (i2 = 0 ; i2 < nbclasses; i2 ++)//Parcours des candidats
				{
					if(class_vect[i2][i1] != NULL)//case non vide
					{
// 						cout << "-- i2 : " <<i2<<" --" << endl ;
						Pk = *(class_vect[i2][i1]);
						/*site0 = new apMHMeSite;
						*site0 = *class_vect[i2][i1];
						vectCand[i2] = site0;*/
						vectCand[i2] = class_vect[i2][i1];//memorise the pointer to the site
						classeMin = 0;
						dmin = 300;

						//boucle sur les classes
						//!Compute distance from the point to every good line
						for (int j = 0; j < nbclasses ; j++)
						{
							if (class_size[j] > (int)(rate_nbpts_min*nbGoodSites))//!We consider only the valid lines, computed with more than nbpts_min points.
							{
								co = cos(mean_lines[j]->getTheta());
								si = sin(mean_lines[j]->getTheta());
								vpPixelMeterConversion::convertPoint(*cam,Pk.jfloat,Pk.ifloat,x,y);
								//std::cout  <<" j "<< j << " loop mean line a " << mean_lines[j]->a << " mean line b " << mean_lines[j]->b << " mean line c " << mean_lines[j]->c << std::endl;
								//!Compute distance from the point to the line j
								d = fabs(mean_lines[j]->getRho()-(x*co+y*si));
								//dgrad = abs(abs(Pk.convlt)/mean_grad[j]);
								//std::cout << " convlt " << d << " dgrad " << dgrad << std::endl;
								//d = d + wgrad*dgrad;
								distances[i2][j] = d;
								if (d<dmin)
								{
									dmin = d;
									classeMin = j;
								}
							}
						}
						classmin[i2] = classeMin;//memorise the min class
						if (i2 != classeMin)
							change = true;


						if (testdiff[classeMin] != 0)
							alldiff = false;
						testdiff[classeMin]++;
					}
				}
				std::cout  << " 0 loop mean line a " << mean_lines[0]->a << " mean line b " << mean_lines[0]->b << " mean line c " << mean_lines[0]->c << std::endl;
				//getchar();

// 				cout << "classemin  : "<<endl <<classmin << endl ;
				//!TODO Changements de classe !

				if ((alldiff)&&(change))
				{
					try
					{
						int a =  2;
						if (nbclasses_corr < numberOfCandidates[i1])
						 throw a;
					}
					catch(...)
					{
						cout << "############### error in find lines !#################" << endl;
						cout << "%%-- i1 : " <<i1<<" ----" << endl ;
						cout << "Test Diff : " <<endl << testdiff << endl;
						cout << "Distance : " <<endl << distances << endl;
						cout<<endl;
						cout << "##################################################" << endl;
					}
	// 				cout << "-- Try change... --" << endl ;
					for (int k = 0; k < nbclasses ; k++)
					{
						if (classmin[k] != -1)
						{//"non vide"
							class_vect[classmin[k]][i1] = vectCand[k];
						}
						if (testdiff[k] == 0)//nouvelles classes vides
							class_vect[k][i1] = NULL;
					}
					nbchange++;
	// 				cout << "-- Change done ! --" << endl ;
				}
				else if (change)
				{
// 					cout << "%%%%%%% NOT ALL DIFFERENTS %%%%%%" <<endl;
// 					cout << "Distance : "<<endl <<distances << endl;
					double dmin_temp = 300;
					int idmin_temp = -1;
					for (int k = 0; k < nbclasses ; k++)
					{
						if (testdiff[k] > 1)
						{
// 							cout<< " hop "<<endl;
							for (int j = 0; j < nbclasses ; j++)
							{
								if (classmin[j]== k)
								{
									if (distances[j][k] < dmin_temp)
									{
										dmin_temp = distances[j][k];
										idmin_temp = j;
									}
								}
							}
							//!changement :
							if (k!=idmin_temp)
							{
								class_vect[k][i1] = vectCand[idmin_temp];
								class_vect[idmin_temp][i1] = vectCand[k];
								nbchange++;
							}
						}
					}
				}
	// 			double t5 = vpTime::measureTimeMs();
	// 			cout << "time point i1 : " << t5-t4 << endl;
			}

	// 		double t7 = vpTime::measureTimeMs();
	// 		cout << "time parcours points : " << t7-t2 << endl;
			//!Compute mean line
			//! and error
	// 		cout << "Compute new mean lines... " << endl ;
            apMeLine curr_line_;
			for (i2 = 0 ; i2 < nbclasses ; i2++)
			{
				//mean_grad_ = 0;
				if (class_size [i2] > (int)(rate_nbpts_min*nbGoodSites)){
					vect_temp.clear();
					for (i1 = 0 ; i1 < nbGoodSites ; i1++)
					{
						if (class_vect[i2][i1] != NULL)
							vect_temp.push_back(*(class_vect[i2][i1]));
						    //mean_grad_ += abs(class_vect[i2][i1]->convlt);
					}
					//curr_line_ = new apMeLine;
					err[i2] = leastSquareLine(I,vect_temp, *cam, curr_line_, rho_m, theta_m);
// 					cout << "err[i2] " << err[i2] << endl;
					if (err[i2] > err_max)
						err_max = err[i2];
					if (err[i2] < err_min)
						err_min = err[i2];
					*(mean_lines[i2]) = curr_line_;
					//mean_grad[i2] = mean_grad_/nbGoodSites;
				}
			}

			std::cout  << " 1 loop mean line a " << mean_lines[0]->a << " mean line b " << mean_lines[0]->b << " mean line c " << mean_lines[0]->c << std::endl;

	// 		cout<<endl;

	// 		double t8 = vpTime::measureTimeMs();
	// 		cout << "time compute new mean lines " << t8-t7 << endl;

			////!display classification
			//std::cout << "K-mean step "<<std::endl;
	 	//	col = vpColor::black;
	 	//	vpDisplay::display(I);
	 	//	for(unsigned int i(0);i<nbclasses;++i){
	 	//		//!Display line
	 	//		col = vpColor::black;
	 	//		mean_lines[i].display(I,col,2);
	 	//		vpDisplay::flush(I);
	 	//
	 	//		//!Display points
	 	//		if (i == 0)
	 	//			col = vpColor::green;
	 	//		else if (i == 1)
	 	//			col = vpColor::blue;
	 	//		else if (i == 2)
	 	//			col = vpColor::red;
			//	else if (i == 3)
 		//			col = vpColor::black;
	 	//		for(i1=0;i1 < nbGoodSites; i1++)
	 	//		{
	 	//			if (class_vect[i][i1] != NULL)
	 	//				class_vect[i][i1]->display(I,col,7);
	 	//		}
	 	//	}
	 	//	vpDisplay::flush(I);
	 	//	vpDisplay::getClick(I);
			//vpDisplay::getImage(I,ItmpGet);
			//if( iter ==1)
			//	vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinitstep1.ppm");
			//else if( iter ==2)
			//	vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinitstep2.ppm");
			//else if( iter ==3)
			//	vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinitstep3.ppm");
	// 		double t3 = vpTime::measureTimeMs();
	// 		cout << "Loop time : " << t3-t2 << endl;
		}//end loop
// 		cout << "End loop Kmean" << endl;
// 		cout << "error " <<endl << err << endl;


		//!*************************************************************************
		//!------------------------------- Results----------------------------------
		//!*************************************************************************

		weight_cumul.clear();
		double weight_cum = 0;

		double dmin_;
		double dmax_;
        double dist_;
		//!display classification and fill features_vect
// 		vpDisplay::getClick(I);

		col = vpColor::black;
// 		vpDisplay::display(I);
		points_vect.resize(nbclasses_corr);

		int j = 0;
		for(unsigned int i(0);i<nbclasses;++i){
			if (class_size [i] > (int)(rate_nbpts_min*nbGoodSites)){
				apMeLine line_tmp ;
				//line_tmp =  this->line;
				line_tmp.a = mean_lines[i]->a;
				line_tmp.b = mean_lines[i]->b;
				line_tmp.c = mean_lines[i]->c;
			    line_tmp.display(I,vpColor::black);
				//rho ??
				//line_tmp.cam = *cam;
				//std::cout << " line a " << line_tmp.a << " line b " << line_tmp.b << " line c " << line_tmp.c << std::endl;

				//!Display points and fill vectors
				//if (i == 0)
				//	col = vpColor::green;
				//else if (i == 1)
				//	col = vpColor::blue;
				//else if (i == 2)
				//	col = vpColor::red;
			    dmin_ = 300;
			    dmax_ = 0;

				for(i1=0;i1 < nbGoodSites; i1++)
				{
					if (class_vect[i][i1] != NULL){

						//if (display_mode == mbtCadModel::ALL) class_vect[i][i1]->display(I,col,5);
						Pk = *(class_vect[i][i1]);
						co = cos(mean_lines[i]->getTheta());
						si = sin(mean_lines[i]->getTheta());
						vpPixelMeterConversion::convertPoint(*cam,Pk.jfloat,Pk.ifloat,x,y);
						dcand =fabs(mean_lines[i]->getRho()-(x*co+y*si));
						class_vect[i][i1]->setWeight(dcand);
						if (dcand < dmin_)
							dmin_ = dcand;
						if (dcand > dmax_)
							dmax_ = dcand;

					}
				}

				for(i1=0;i1 < nbGoodSites; i1++)
				{
					if (class_vect[i][i1] != NULL){

						dist_ = class_vect[i][i1]->getWeight();
						class_vect[i][i1]->setWeight((dist_-dmin_)*(dist_-dmin_)/((dmax_-dmin_)*(dmax_-dmin_)));
						points_vect[j].push_back(*class_vect[i][i1]);
						line_tmp.list.push_back(*class_vect[i][i1]);
					}
				}


				//!Weight :
				double wght;
				/*if (err_min > 0.7)
					wght = 0.1;
				else*/
					if (err_min == err_max )
					{
						wght = exp(-(err[i])*(err[i])/0.02);
						//std::cout << "wght : " << wght << std::endl;
					}

				else
					wght = exp(-(err[i]-err_min)*(err[i]-err_min)/((err_max-err_min)*(err_max-err_min)));
// 					wght = exp(-(err[i])*(err[i])/0.02);
// 				double wght = err.euclideanNorm()/(float)err.getHeight();
				if (wght <= 0.00001) wght = 0.00001;

				std::cout << " weight line " << i << " : " << wght << std::endl;

				weights.push_back(wght);//!may need to be adjusted ?
// 				cout << "err["<< i <<"] = " <<fabs(err[i]) << endl;

				unsigned int colval = (int)255*(1-wght);
				weight_cum += wght;
				weight_cumul.push_back(weight_cum);
				//line_tmp.weight_cumul = weight_cumul;

// 				cout << "weight : " <<feature.weight << endl;
// 				cout << "colval : " <<colval << endl;


				//lines_vect.push_back(line_tmp);

				  vpImagePoint ip;


				  for (int l =0; l<list.size(); l++)
				  {
				    apMHMeSite p = line_tmp.list[l] ;

				    if(p.suppress == 1) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
				      //vpDisplay::displayCross(I, ip, 2, vpColor::white) ; // Contrast
				    }
				    else if(p.suppress == 2) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
				      //vpDisplay::displayCross(I, ip, 2,vpColor::blue) ; // Threshold
				    }
				    else if(p.suppress == 3) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
				      //vpDisplay::displayCross(I, ip, 3, vpColor::green) ; // M-estimator
				    }
				    else if(p.suppress == 0) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
					  if(i == 0)
					  {
				      vpDisplay::displayCross(I, ip, 2, vpColor::blue) ; // OK
					  }
					  if(i == 1)
					  {
				      vpDisplay::displayCross(I, ip, 2, vpColor::yellow) ; // OK
					  }
					  if(i == 2)
					  {
				      vpDisplay::displayCross(I, ip, 2, vpColor::purple) ; // OK
					  }
				    }
				  }


				//!Display line
				//Display line according to its weight


				//col.setColor(colval,colval,colval);

				 // Draw a red rectangle in the display overlay (foreground)
// 				((vpDisplayX*)(I.display))->displayLine( 100, 10, 80, 400, col, 2);

				//line_tmp.display(I,col);

				//vpDisplay::flush(I);
 			//	vpDisplay::getClick(I);

// 				if (display_mode == mbtCadModel::ALL)
// 				{
 //// 					vpRGBa greyCol(colval);
 //					col = vpColor::black;
 //// 					mean_lines[i].display(I,col);
 //					mean_lines[i].display(I,col);
 //					vpDisplay::flush(I);
// 				}


				j++;
			}
		}

		for (i2 = 0 ; i2 < nbclasses ; i2++)
		{
			curr_line = mean_lines[i2];
			delete curr_line;
		}

// 		cout << "err_min : " << err_min << endl;
// 		cout << "err_max : " << err_max << endl;
		//vpDisplay::flush(I);
		//!TEST
// 		vpHomogeneousMatrix cMo;
// 		features_vect[0].line.Project(cMo);
		//!
// 		vpDisplay::getClick(I);

		//double t2 = vpTime::measureTimeMs();
		//std::cout <<"******* find lines time: " << t2-t1<<std::endl;

// 		DisplayListAll(I,vpColor::red,3);
		//if (DEBUG_LEVEL1)
			std::cout << "end vpMeTracker::findLinesByKmean() " << std::endl;
		return j;
	}
	else
	{
		//double t2 = vpTime::measureTimeMs();
		//std::cout <<"******* find lines time: " << t2-t1<<std::endl;
		//no candidate for that feature (not enough points)
		//if (DEBUG_LEVEL1)
			std::cout << "end vpMeTracker::findLinesByKmean() (not enough points)" << std::endl ;
		return 0;
	}


}


void
apMHMeLine::setParameterSpace(int &n, int &m, const vpImage<unsigned char> &I,vpCameraParameters &cam)
{
//V.init(n,m);
Theta.resize(n);
Rho.resize(m);
int h,w;
double r;
h=I.getHeight();
w=I.getWidth();
r=sqrt((double)h*h/(cam.get_py()*cam.get_py())+(double)w*w/(cam.get_px()*cam.get_px()));
//Rho.resize(r);
//LineVote.init(n,m);
//Rho[0]=0;
int i,j;
for (i=0;i<n;i++)
{
Theta[i]=i*M_PI/n-M_PI/2;
}
for (j=0;j<m;j++)
{
Rho[j]=j*r/m-r/2;
}
}

/*!
	\brief : Search for lines from vpSites, using Kmean classification approach.

 \param I : Image in which  to display
 \param display_mode : Display mode
 \return : number of found lines

*/
int apMHMeLine::findLinesByKmean_2(const vpImage< unsigned char> &I,int display_mode)
{
	//double t1 = vpTime::measureTimeMs();

	//if (DEBUG_LEVEL1)
		std::cout << "begin apMHMeLine::findLinesByKmean() " << std::endl ;
// 	cout << "----------------------------------------"<< endl;
	points_vect.clear();
	//lines_vect.clear();
	weights.clear();


// 	double t0 = vpTime::measureTimeMs();
	//!Init parameters
	int nbGdSites_min = 3;
	bool not_finished = true;
	unsigned int iter = 0;
	int n_hyp = (&(list[0]))->getNumCandidates();

	//std::cout << "n_hyp : " <<n_hyp<<std::endl;
//n_hyp=2;
	float error = 0;

	int nbclasses = 3;
	int nbIterMax = 1;
	float rate_nbpts_min = 0.4;

        vpColVector class_size;
	vpColVector numberOfCandidates;

	class_size.resize(nbclasses);
	vpColVector err;

	float err_min = 9999999;
	float err_max = 0;

	//!*************************************************************************
	//!--------------------------Initialisation---------------------------------
	//!*************************************************************************

	//!copy good points in a vector and get number of classes (max number of good candidates)
// 	std::cout << "initialise vectors" << std::endl;
	int nbGoodSites = 0;
	std::vector< std::vector<apMHMeSite> > site_vect;
	apMHMeSite Pk;
	apMHMeSite Pk0;
    std::vector<apMHMeSite> Pk1(n_hyp);
	apMHMeSite Pk2;
	int i1, i2 ;
	i1 = i2 = 0;
	for (int l =0; l<list.size(); l++)
	{
		if (list[l].suppress == 0)
		{
			nbGoodSites++;
			site_vect.resize(nbGoodSites);
			i2=0;
			for (int k = 0; k < n_hyp ; k++) //for each candidate of P
			{
				Pk = ((&(list[l]))->candidateList[k]);
				//if((Pk.suppress == 0) )
				{

					site_vect[i1].push_back(Pk);
					//std::cout << " ii " << site_vect[i1][i2].i << " jj " << site_vect[i1][i2].j << " gradmap " << site_vect[i1][i2].thetaGrad << std::endl;
					i2++;
					if(i2>nbclasses)
					{
						nbclasses = i2;
					}
				}
			}
			i1++;
		}
	}
	int nbGoodSiteTotal = numberOfSignal_MH();
	err.resize(nbclasses);
	err = 0;

	std::cout << "nbGoodSites : " << nbGoodSites << std::endl;

	if (nbGoodSites > nbGdSites_min)
	{
 		/*std::cout << "-- nbclasses : " << nbclasses << std::endl;
 		std::cout << "nbGoodSites : " << nbGoodSites << std::endl;
 		std::cout << "nbGoodSiteTotal : " << nbGoodSiteTotal << std::endl;
	 	std::cout << "size site_vect : " << site_vect.size() << std::endl;*/
		class_size.resize(nbclasses);
		numberOfCandidates.resize(nbGoodSites);
		numberOfCandidates = 0;

		//!Construct the class_vect tab in which the process will be made
		//!Also init class_size vect and numberOfCandidates vect in the same time
		std::vector< std::vector<apMHMeSite*> > class_vect;
		apMHMeSite* site;
		class_vect.resize(nbclasses);
		for (i2 = 0 ; i2 < nbclasses ; i2++)
			class_vect[i2].resize(nbGoodSites);

		for (i1 = 0 ; i1 < nbGoodSites ; i1++)
		{
			for (i2 = 0 ; i2 < nbclasses ; i2++)
			{
				numberOfCandidates[i1] = site_vect[i1].size();
				if (i2 < site_vect[i1].size())
				{
					/*site = new apMHMeSite;
					*site = site_vect[i1][i2];
					class_vect[i2][i1] = site;*/
					class_vect[i2][i1] = &(site_vect[i1][i2]);
					class_size[i2] ++;
				}

				else
					class_vect[i2][i1] = NULL;
			}
		}

		////!display classification
 		vpColor col = vpColor::black;
 	//	vpDisplay::display(I);
 	//	for(unsigned int i(0);i<nbclasses;++i){
		//
 	//		//!Display points
 	//		if (i == 0)
 	//			col = vpColor::green;
 	//		else if (i == 1)
 	//			col = vpColor::blue;
 	//		else if (i == 2)
 	//			col = vpColor::red;
		//	else if (i == 3)
 	//			col = vpColor::black;
 	//		for(i1=0;i1 < nbGoodSites; i1++)
 	//		{
 	//			if (i < site_vect[i1].size())
 	//				site_vect[i1][i].display(I,col,7);
 	//		}
 	//	}
 	//	vpDisplay::flush(I);
 	//	vpDisplay::getClick(I);
		//vpImage<vpRGBa> ItmpGet;
		//vpDisplay::getImage(I,ItmpGet);
		//vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinit.ppm");


	// 	std::cout << "class vect init ok " << std::endl;
// 		std::cout << "-- " << std::endl;
// 		for (i1 = 0 ; i1 < nbGoodSites ; i1++)
// 		{
// 			std::cout << "nOfCand "<< i1 << " : "<< numberOfCandidates[i1] << " , " ;
// 		}
// 		std::cout << std::endl << "-- " << std::endl;

		//! nbclass_corr considers only the classes with more than nbpts_min candidates.
		int nbclasses_corr = 0;
		for (int i = 0 ; i < nbclasses; i++)
		{
			if (class_size[i] > (int)(rate_nbpts_min*nbGoodSites))
			{
				nbclasses_corr ++;
			}
		}
// 		std::cout << "-- nbclasses long enough : " << nbclasses_corr << std::endl;
// 		std::cout << "-classe size : " <<endl<< class_size << std::endl;


		//!Vectors to memorise lines (result):
		std::vector<apMeLine*> mean_lines;
		mean_lines.resize(nbclasses);
		std::vector<double> mean_grad;
		mean_grad.resize(nbclasses);
		double grad;
		double mean_grad_ = 0;

		apMeLine *curr_line;
		double theta_m, rho_m;
		double co, si;
		double x,y;

		int ms,ns;
		double thetas;
		int indTheta;
		int indRho;
		int thetaSpace = 180;
		int rhoSpace = 600;
		setParameterSpace(thetaSpace,rhoSpace,I, *cam);
		int height,width;
		height=I.getHeight();
		width=I.getWidth();
		double rhol =floor(sqrt(height*height+width*width));
		std::vector<couple> couples;
        LineVote.init(thetaSpace,rhoSpace);
		int start,end;

		std::vector<apMHMeSite> vect_temp;
        double xx,yy;

		//!init means.
		for (i2 = 0 ; i2 < nbclasses ; i2++)
		{
			//mean_grad_ = 0;
			if (class_size [i2] > (int)(rate_nbpts_min*nbGoodSites))
			{
				vect_temp.clear();
				for (i1 = 0 ; i1 < nbGoodSites ; i1++)
				{
					if (class_vect[i2][i1] != NULL){
						//vect_temp.push_back(*(class_vect[i2][i1]));
					    //mean_grad_ += abs(class_vect[i2][i1]->convlt);
					ns = class_vect[i2][i1]->i;
					ms = class_vect[i2][i1]->j;
					thetas = class_vect[i2][i1]->thetaGrad+M_PI/2;
					indTheta = floor(thetas/(M_PI/thetaSpace));
					if (indTheta==thetaSpace)
						indTheta--;
					/*start = indTheta-1;
					end = indTheta+1;
					if(indTheta<1)
						start = 0;
					if(indTheta>Theta.size()-1)
						end = Theta.size();
			        for (int ii=start;ii<end;ii++)*/
			        //{
					vpPixelMeterConversion::convertPoint(*cam,ms,ns,xx,yy);
					rho = xx*cos(Theta[indTheta])+yy*sin(Theta[indTheta]) - Rho[0];
			        indRho = floor(rho/(-2*Rho[0]/rhoSpace));
			        LineVote.VoteMatrix[indTheta][indRho]=LineVote.VoteMatrix[indTheta][indRho]+1;
			        //}
				}
				}
				/*curr_line = new apMeLine;
				leastSquareLine(I,vect_temp, *cam, *curr_line, rho_m, theta_m);
				mean_lines[i2] = curr_line;*/
				//mean_grad[i2] = (mean_grad_)/nbGoodSites;
				//std::cout << " i2 "<< i2 << " mean line a " << mean_lines[i2]->a << " mean line b " << mean_lines[i2]->b << " mean line c " << mean_lines[i2]->c << std::endl;
			}
		}
		//std::cout << "mean_lines init ok " << std::endl;
		double aa,bb,cc;
		couples = LineVote.getBestVotes(nbclasses);
		for (i2 = 0 ; i2 < nbclasses ; i2++)
		{
			curr_line = new apMeLine;
			aa = cos(Theta[couples[i2].i1]);
			bb = sin(Theta[couples[i2].i1]);
			cc = -Rho[couples[i2].j1];
			curr_line->InitLine(aa,bb,cc);

			//vpFeatureDisplay::displayLine(Rho[couples[i2].j1],Theta[couples[i2].i1], *cam, I, vpColor::yellow, 2) ;
			//std::cout << " aa "<< curr_line->getRho() << " bb "<< curr_line->getTheta() << " cc "<< cc << std::endl;
			mean_lines[i2] = curr_line;
		}
		//std::cout << " votematrix "<< couples[0].vote << std::endl;
	// 	double t1 = vpTime::measureTimeMs();
	// 	vpDisplay::flush(I);
	// 	vpDisplay::getClick(I);
	// 	cout << "Time init : " << t1-t0 <<endl;


		////!display classification
		//std::cout << "K-mean init "<<std::endl;
 	//	col = vpColor::black;
 	//	vpDisplay::display(I);
 	//	for(unsigned int i(0);i<nbclasses;++i){
 	//		//!Display line
 	//		col = vpColor::black;
		//	/*if (class_size [i] > (int)(rate_nbpts_min*nbGoodSites))*/
 	//			mean_lines[i].display(I,col,2);
 	//		vpDisplay::flush(I);
 	//
 	//		//!Display points
 	//		if (i == 0)
 	//			col = vpColor::green;
 	//		else if (i == 1)
 	//			col = vpColor::blue;
 	//		else if (i == 2)
 	//			col = vpColor::red;
		//	else if (i == 3)
 	//			col = vpColor::black;
 	//		for(i1=0;i1 < nbGoodSites; i1++)
 	//		{
 	//			if (class_vect[i][i1] != NULL)
 	//				class_vect[i][i1]->display(I,col,7);
 	//		}
 	//	}
 	//	vpDisplay::flush(I);
 	//	vpDisplay::getClick(I);
		//
		//vpDisplay::getImage(I,ItmpGet);
		//vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinit2.ppm");

	//!*************************************************************************
	//!-------------------------------LOOP--------------------------------------
	//!*************************************************************************
		std::vector< std::vector<apMHMeSite> >::iterator it1;
		std::vector<apMHMeSite>::iterator it2;

		//std::cout  << " 0 loop mean line a " << mean_lines[0]->a << " mean line b " << mean_lines[0]->b << " mean line c " << mean_lines[0]->c << std::endl;

		int nbchange=1 ;
		bool change = false;
		int iteration=0 ;
		int classeMin = 0;
		double dmin = 300;
	// 	double dminAllCand = 300;
	// 	int i2MinAllCand;
	// 	int classMinAllCand;
		double d = 0;
		vpMatrix distances (nbclasses,nbclasses);
		vpColVector classmin(nbclasses);
		vpColVector testdiff(nbclasses);
		bool alldiff = true;
		std::vector<apMHMeSite*> vectCand;
		vectCand.resize(nbclasses);
		//apMHMeSite* site0;
		double dcand;
		double dgrad=0;
		double wgrad = 1/2;
// 		cout << "Start loop Kmean" << endl;
		while((nbchange!=0)&&(iter < nbIterMax))
		{
			iter++;
			double t2 = vpTime::measureTimeMs();
	// 		cout << "*** " <<iteration++<<" ***" << endl ;
			nbchange  = 0;
			mean_grad_ = 0;
			i1 = i2 = 0;
			//loop on the points
			for (i1 = 0 ; i1 < nbGoodSites ; i1++)
			{
	// 			double t4 = vpTime::measureTimeMs();
				i2 = 0;
				change = false;
				classmin = -1;
				testdiff = 0;
				distances = -1;
				alldiff = true;
// 				cout << "%%-- i1 : " <<i1<<" ----" << endl ;

				//!For each candidate, compute its distance to each valid line and make class permutations
				for (i2 = 0 ; i2 < nbclasses; i2 ++)//Parcours des candidats
				{
					if(class_vect[i2][i1] != NULL)//case non vide
					{
// 						cout << "-- i2 : " <<i2<<" --" << endl ;
						Pk = *(class_vect[i2][i1]);
						/*site0 = new apMHMeSite;
						*site0 = *class_vect[i2][i1];
						vectCand[i2] = site0;*/
						vectCand[i2] = class_vect[i2][i1];//memorise the pointer to the site
						classeMin = 0;
						dmin = 300;

						//boucle sur les classes
						//!Compute distance from the point to every good line
						for (int j = 0; j < nbclasses ; j++)
						{
							if (class_size[j] > (int)(rate_nbpts_min*nbGoodSites))//!We consider only the valid lines, computed with more than nbpts_min points.
							{
								co = cos(mean_lines[j]->getTheta());
								si = sin(mean_lines[j]->getTheta());
								vpPixelMeterConversion::convertPoint(*cam,Pk.jfloat,Pk.ifloat,x,y);
								//std::cout  <<" j "<< j << " loop mean line a " << mean_lines[j]->a << " mean line b " << mean_lines[j]->b << " mean line c " << mean_lines[j]->c << std::endl;
								//!Compute distance from the point to the line j
								d = fabs(mean_lines[j]->getRho()-(x*co+y*si));
								//dgrad = abs(abs(Pk.convlt)/mean_grad[j]);
								//std::cout << " convlt " << d << " dgrad " << dgrad << std::endl;
								//d = d + wgrad*dgrad;
								distances[i2][j] = d;
								if (d<dmin)
								{
									dmin = d;
									classeMin = j;
								}
							}
						}
						classmin[i2] = classeMin;//memorise the min class
						if (i2 != classeMin)
							change = true;


						if (testdiff[classeMin] != 0)
							alldiff = false;
						testdiff[classeMin]++;
					}
				}
				//std::cout  << " 0 loop mean line a " << mean_lines[0]->a << " mean line b " << mean_lines[0]->b << " mean line c " << mean_lines[0]->c << std::endl;
				//getchar();

// 				cout << "classemin  : "<<endl <<classmin << endl ;
				//!TODO Changements de classe !

				if ((alldiff)&&(change))
				{
					try
					{
						int a =  2;
						if (nbclasses_corr < numberOfCandidates[i1])
						 throw a;
					}
					catch(...)
					{
						cout << "############### error in find lines !#################" << endl;
						cout << "%%-- i1 : " <<i1<<" ----" << endl ;
						cout << "Test Diff : " <<endl << testdiff << endl;
						cout << "Distance : " <<endl << distances << endl;
						cout<<endl;
						cout << "##################################################" << endl;
					}
	// 				cout << "-- Try change... --" << endl ;
					for (int k = 0; k < nbclasses ; k++)
					{
						if (classmin[k] != -1)
						{//"non vide"
							class_vect[classmin[k]][i1] = vectCand[k];
						}
						if (testdiff[k] == 0)//nouvelles classes vides
							class_vect[k][i1] = NULL;
					}
					nbchange++;
	// 				cout << "-- Change done ! --" << endl ;
				}
				else if (change)
				{
// 					cout << "%%%%%%% NOT ALL DIFFERENTS %%%%%%" <<endl;
// 					cout << "Distance : "<<endl <<distances << endl;
					double dmin_temp = 300;
					int idmin_temp = -1;
					for (int k = 0; k < nbclasses ; k++)
					{
						if (testdiff[k] > 1)
						{
// 							cout<< " hop "<<endl;
							for (int j = 0; j < nbclasses ; j++)
							{
								if (classmin[j]== k)
								{
									if (distances[j][k] < dmin_temp)
									{
										dmin_temp = distances[j][k];
										idmin_temp = j;
									}
								}
							}
							//!changement :
							if (k!=idmin_temp)
							{
								class_vect[k][i1] = vectCand[idmin_temp];
								class_vect[idmin_temp][i1] = vectCand[k];
								nbchange++;
							}
						}
					}
				}
	// 			double t5 = vpTime::measureTimeMs();
	// 			cout << "time point i1 : " << t5-t4 << endl;
			}

	// 		double t7 = vpTime::measureTimeMs();
	// 		cout << "time parcours points : " << t7-t2 << endl;
			//!Compute mean line
			//! and error
	// 		cout << "Compute new mean lines... " << endl ;
            apMeLine curr_line_;
			for (i2 = 0 ; i2 < nbclasses ; i2++)
			{
				//mean_grad_ = 0;
				if (class_size [i2] > (int)(rate_nbpts_min*nbGoodSites)){
					vect_temp.clear();
					for (i1 = 0 ; i1 < nbGoodSites ; i1++)
					{
						if (class_vect[i2][i1] != NULL)
							vect_temp.push_back(*(class_vect[i2][i1]));
						    //mean_grad_ += abs(class_vect[i2][i1]->convlt);
					}
					//curr_line_ = new apMeLine;
					err[i2] = leastSquareLine(I,vect_temp, *cam, curr_line_, rho_m, theta_m);
// 					cout << "err[i2] " << err[i2] << endl;
					if (err[i2] > err_max)
						err_max = err[i2];
					if (err[i2] < err_min)
						err_min = err[i2];
					*(mean_lines[i2]) = curr_line_;
					//mean_grad[i2] = mean_grad_/nbGoodSites;
				}
			}

			//std::cout  << " 1 loop mean line a " << mean_lines[0]->a << " mean line b " << mean_lines[0]->b << " mean line c " << mean_lines[0]->c << std::endl;

	// 		cout<<endl;

	// 		double t8 = vpTime::measureTimeMs();
	// 		cout << "time compute new mean lines " << t8-t7 << endl;

			////!display classification
			//std::cout << "K-mean step "<<std::endl;
	 	//	col = vpColor::black;
	 	//	vpDisplay::display(I);
	 	//	for(unsigned int i(0);i<nbclasses;++i){
	 	//		//!Display line
	 	//		col = vpColor::black;
	 	//		mean_lines[i].display(I,col,2);
	 	//		vpDisplay::flush(I);
	 	//
	 	//		//!Display points
	 	//		if (i == 0)
	 	//			col = vpColor::green;
	 	//		else if (i == 1)
	 	//			col = vpColor::blue;
	 	//		else if (i == 2)
	 	//			col = vpColor::red;
			//	else if (i == 3)
 		//			col = vpColor::black;
	 	//		for(i1=0;i1 < nbGoodSites; i1++)
	 	//		{
	 	//			if (class_vect[i][i1] != NULL)
	 	//				class_vect[i][i1]->display(I,col,7);
	 	//		}
	 	//	}
	 	//	vpDisplay::flush(I);
	 	//	vpDisplay::getClick(I);
			//vpDisplay::getImage(I,ItmpGet);
			//if( iter ==1)
			//	vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinitstep1.ppm");
			//else if( iter ==2)
			//	vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinitstep2.ppm");
			//else if( iter ==3)
			//	vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinitstep3.ppm");
	// 		double t3 = vpTime::measureTimeMs();
	// 		cout << "Loop time : " << t3-t2 << endl;
		}//end loop
// 		cout << "End loop Kmean" << endl;
// 		cout << "error " <<endl << err << endl;


		//!*************************************************************************
		//!------------------------------- Results----------------------------------
		//!*************************************************************************

		weight_cumul.clear();
		double weight_cum = 0;

		double dmin_;
		double dmax_;
        double dist_;
		//!display classification and fill features_vect
// 		vpDisplay::getClick(I);

		col = vpColor::black;
// 		vpDisplay::display(I);
		points_vect.resize(nbclasses);
		int j = 0;
		for(unsigned int i(0);i<nbclasses;++i){
			if (class_size [i] > (int)(rate_nbpts_min*nbGoodSites)){
				apMeLine line_tmp ;
				//line_tmp =  this->line;
				line_tmp.a = mean_lines[i]->a;
				line_tmp.b = mean_lines[i]->b;
				line_tmp.c = mean_lines[i]->c;
			    line_tmp.display(I,vpColor::black);

			    /*if(i==2)
			    vpFeatureDisplay::displayLine(mean_lines[i]->getRho(),mean_lines[i]->getTheta(), *cam, I, vpColor::purple, 1) ;
			    if(i==1)
			    vpFeatureDisplay::displayLine(mean_lines[i]->getRho(),mean_lines[i]->getTheta(), *cam, I, vpColor::red, 1) ;
			    if(i==0)
			    vpFeatureDisplay::displayLine(mean_lines[i]->getRho(),mean_lines[i]->getTheta(), *cam, I, vpColor::blue, 1) ;
				//rho ??
				//line_tmp.cam = *cam;
				std::cout << " line a " << line_tmp.a << " line b " << line_tmp.b << " line c " << line_tmp.c << std::endl;*/

				//!Display points and fill vectors
				//if (i == 0)
				//	col = vpColor::green;
				//else if (i == 1)
				//	col = vpColor::blue;
				//else if (i == 2)
				//	col = vpColor::red;
			    dmin_ = 300;
			    dmax_ = 0;

				for(i1=0;i1 < nbGoodSites; i1++)
				{
					if (class_vect[i][i1] != NULL){

						//if (display_mode == mbtCadModel::ALL) class_vect[i][i1]->display(I,col,5);
						Pk = *(class_vect[i][i1]);
						co = cos(mean_lines[i]->getTheta());
						si = sin(mean_lines[i]->getTheta());
						vpPixelMeterConversion::convertPoint(*cam,Pk.jfloat,Pk.ifloat,x,y);
						dcand =fabs(mean_lines[i]->getRho()-(x*co+y*si));
						class_vect[i][i1]->setWeight(dcand);
						if (dcand < dmin_)
							dmin_ = dcand;
						if (dcand > dmax_)
							dmax_ = dcand;

					}
				}




					//vpFeatureDisplay::displayLine(Rho[couples[i2].j1],Theta[couples[i2].i1], *cam, I, vpColor::yellow, 2) ;
					//std::cout << " aa "<< Rho[couples[i2].j1] << " bb "<< Theta[couples[i2].i1] << " cc "<< cc << std::endl;
				   curr_line = mean_lines[i];
					delete curr_line;


				//!Weight :
				double wght;
				/*if (err_min > 0.7)
					wght = 0.1;
				else*/
					if (err_min == err_max )
					{
						wght = exp(-(err[i])*(err[i])/0.02);
						//std::cout << "wght : " << wght << std::endl;
					}

				else
					wght = exp(-(err[i]-err_min)*(err[i]-err_min)/((err_max-err_min)*(err_max-err_min)));
// 					wght = exp(-(err[i])*(err[i])/0.02);
// 				double wght = err.euclideanNorm()/(float)err.getHeight();
				if (wght <= 0.00001) wght = 0.00001;

				//std::cout << " weight line " << i << " : " << wght << std::endl;

				weights.push_back(wght);//!may need to be adjusted ?
// 				cout << "err["<< i <<"] = " <<fabs(err[i]) << endl;

				double mean = 0;
				int km =0;
				double wghtm;

				for(i1=0;i1 < nbGoodSites; i1++)
				{
					if (class_vect[i][i1] != NULL){

						dist_ = class_vect[i][i1]->getWeight();
						if (dmin_ == dmax_)
						{
							wghtm = wght*exp(-(dist_-dmin_)*(dist_-dmin_)/0.0001);
						}
						else
						{
							//wghtm = wght*exp(-(dist_-dmin_)*(dist_-dmin_)/((dmax_-dmin_)*(dmax_-dmin_)));
							wghtm = wght*exp(-(dist_-dmin_)*(dist_-dmin_)/0.001);
						}
						mean = mean + wghtm;

					}
				}
				mean /= nbGoodSites;

				for(i1=0;i1 < nbGoodSites; i1++)
				{
					if (class_vect[i][i1] != NULL){

						dist_ = class_vect[i][i1]->getWeight();
						if (dmin_ == dmax_)
							class_vect[i][i1]->setWeight(wght*exp(-(dist_-dmin_)*(dist_-dmin_)/0.001)/mean);
						else
						class_vect[i][i1]->setWeight(wght*exp(-(dist_-dmin_)*(dist_-dmin_)/((dmax_-dmin_)*(dmax_-dmin_)))/mean);
						points_vect[j].push_back(*class_vect[i][i1]);
						line_tmp.list.push_back(*class_vect[i][i1]);
					}
				}

				unsigned int colval = (int)255*(1-wght);
				weight_cum += wght;
				weight_cumul.push_back(weight_cum);
				//line_tmp.weight_cumul = weight_cumul;

// 				cout << "weight : " <<feature.weight << endl;
// 				cout << "colval : " <<colval << endl;


				//lines_vect.push_back(line_tmp);

				  vpImagePoint ip;

				  for (int l =0; l<list.size(); l++)
				  {
				    apMHMeSite p = line_tmp.list[l] ;

				    if(p.suppress == 1) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
				      //vpDisplay::displayCross(I, ip, 2, vpColor::white) ; // Contrast
				    }
				    else if(p.suppress == 2) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
				      //vpDisplay::displayCross(I, ip, 2,vpColor::blue) ; // Threshold
				    }
				    else if(p.suppress == 3) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
				      //vpDisplay::displayCross(I, ip, 3, vpColor::green) ; // M-estimator
				    }
				    else if(p.suppress == 0) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
					  if(i == 0)
					  {
				      //vpDisplay::displayCross(I, ip, 2, vpColor::blue) ; // OK
					  }
					  if(i == 1)
					  {
				      //vpDisplay::displayCross(I, ip, 2, vpColor::yellow) ; // OK
					  }
					  if(i == 2)
					  {
				      //vpDisplay::displayCross(I, ip, 2, vpColor::purple) ; // OK
					  }
				    }
				  }


				//!Display line
				//Display line according to its weight


				//col.setColor(colval,colval,colval);

				 // Draw a red rectangle in the display overlay (foreground)
// 				((vpDisplayX*)(I.display))->displayLine( 100, 10, 80, 400, col, 2);

				//line_tmp.display(I,col);

				//vpDisplay::flush(I);
 			//	vpDisplay::getClick(I);

// 				if (display_mode == mbtCadModel::ALL)
// 				{
 //// 					vpRGBa greyCol(colval);
 //					col = vpColor::black;
 //// 					mean_lines[i].display(I,col);
 //					mean_lines[i].display(I,col);
 //					vpDisplay::flush(I);
// 				}


				j++;
			}
		}
		/*for (i2 = 0 ; i2 < nbclasses ; i2++)
		{
			curr_line = mean_lines[i2];
			delete curr_line;
		}*/
// 		cout << "err_min : " << err_min << endl;
// 		cout << "err_max : " << err_max << endl;
		//vpDisplay::flush(I);
		//!TEST
// 		vpHomogeneousMatrix cMo;
// 		features_vect[0].line.Project(cMo);
		//!
// 		vpDisplay::getClick(I);

		//double t2 = vpTime::measureTimeMs();
		//std::cout <<"******* find lines time: " << t2-t1<<std::endl;

// 		DisplayListAll(I,vpColor::red,3);
		//if (DEBUG_LEVEL1)
			std::cout << "end vpMeTracker::findLinesByKmean() " << std::endl;
		return j;
	}
	else
	{
		//double t2 = vpTime::measureTimeMs();
		//std::cout <<"******* find lines time: " << t2-t1<<std::endl;
		//no candidate for that feature (not enough points)
		//if (DEBUG_LEVEL1)
			std::cout << "end vpMeTracker::findLinesByKmean() (not enough points)" << std::endl ;
		return 0;
	}


}


int apMHMeLine::findLinesByKmean_3(const vpImage< unsigned char> &I,int display_mode)
{
	//double t1 = vpTime::measureTimeMs();

	//if (DEBUG_LEVEL1)
		std::cout << "begin apMHMeLine::findLinesByKmean() " << std::endl ;
// 	cout << "----------------------------------------"<< endl;
	points_vect.clear();
	//lines_vect.clear();
	weights.clear();


// 	double t0 = vpTime::measureTimeMs();
	//!Init parameters
	int nbGdSites_min = 1;
	bool not_finished = true;
	unsigned int iter = 0;
	int n_hyp = (&(list[0]))->getNumCandidates();

	//std::cout << "n_hyp : " <<n_hyp<<std::endl;
//n_hyp=2;
	float error = 0;

	int nbclasses = 3;
	int nbIterMax = 1;
	bool recomputeline = true;
	float rate_nbpts_min = 0.1;

        vpColVector class_size;
	vpColVector numberOfCandidates;

	class_size.resize(nbclasses);
	vpColVector err;

	float err_min = 9999999;
	float err_max = 0;

	//!*************************************************************************
	//!--------------------------Initialisation---------------------------------
	//!*************************************************************************

	//!copy good points in a vector and get number of classes (max number of good candidates)
// 	std::cout << "initialise vectors" << std::endl;
	int nbGoodSites = 0;
	std::vector< std::vector<apMHMeSite> > site_vect;
	//apMHMeSite Pk;
	apMHMeSite Pk;
	apMHMeSite Pk0;
    std::vector<apMHMeSite> Pk1(n_hyp);
	apMHMeSite Pk2;
	  vpImagePoint ip;
	int i1, i2 ;
	i1 = i2 = 0;
	std::vector<apMHMeSite> site_vect_temp;
	for (int l =0; l<list.size(); l++)
	{
		if (list[l].suppress == 0)
		{
			nbGoodSites++;
			//site_vect.resize(nbGoodSites);
			site_vect_temp.resize(0);
			i2=0;
			//std::cout << " track mh0 " <<  (&(list.value()))->candidateList[0].i<< std::endl;
			for (int k = 0; k < n_hyp ; k++) //for each candidate of P
			{
				Pk = ((&(list[l]))->candidateList[k]);

				//Pk = &(list.value()).candidateVect[k];
				{
					site_vect_temp.push_back(Pk);
					//site_vect[i1].push_back(Pk);
					//std::cout << " ii " << site_vect[i1][i2].i << " jj " << site_vect[i1][i2].j << " gradmap " << site_vect[i1][i2].thetaGrad << std::endl;
					i2++;
					if(i2>nbclasses)
					{
						nbclasses = i2;
					}
				}
			}
			site_vect.push_back(site_vect_temp);
			i1++;
		}
	}
	int nbGoodSiteTotal = numberOfSignal_MH();
	err.resize(nbclasses);
	err = 0;

	if (nbGoodSites > nbGdSites_min)
	{
 		/*std::cout << "-- nbclasses : " << nbclasses << std::endl;
 		std::cout << "nbGoodSites : " << nbGoodSites << std::endl;
 		std::cout << "nbGoodSiteTotal : " << nbGoodSiteTotal << std::endl;
	 	std::cout << "size site_vect : " << site_vect.size() << std::endl;*/
		class_size.resize(nbclasses);
		numberOfCandidates.resize(nbGoodSites);
		numberOfCandidates = 0;

		//!Construct the class_vect tab in which the process will be made
		//!Also init class_size vect and numberOfCandidates vect in the same time
		std::vector< std::vector<apMHMeSite*> > class_vect;
		apMHMeSite* site;
		class_vect.resize(nbclasses);
		for (i2 = 0 ; i2 < nbclasses ; i2++)
			class_vect[i2].resize(nbGoodSites);

		for (i1 = 0 ; i1 < nbGoodSites ; i1++)
		{
			for (i2 = 0 ; i2 < nbclasses ; i2++)
			{
				numberOfCandidates[i1] = site_vect[i1].size();
				//std::cout << " size class " << site_vect[i1].size() << std::endl;
				//if (i2 < site_vect[i1].size())
				{
					/*site = new apMHMeSite;
					*site = site_vect[i1][i2];
					class_vect[i2][i1] = site;*/
					class_vect[i2][i1] = &(site_vect[i1][i2]);
					class_size[i2] ++;

				}

				/*else
					class_vect[i2][i1] = NULL;*/
			}
		}

		////!display classification
		//std::cout << "K-mean init site_vect"<<std::endl;
 		vpColor col = vpColor::black;
 	//	vpDisplay::display(I);
 	//	for(unsigned int i(0);i<nbclasses;++i){
		//
 	//		//!Display points
 	//		if (i == 0)
 	//			col = vpColor::green;
 	//		else if (i == 1)
 	//			col = vpColor::blue;
 	//		else if (i == 2)
 	//			col = vpColor::red;
		//	else if (i == 3)
 	//			col = vpColor::black;
 	//		for(i1=0;i1 < nbGoodSites; i1++)
 	//		{
 	//			if (i < site_vect[i1].size())
 	//				site_vect[i1][i].display(I,col,7);
 	//		}
 	//	}
 	//	vpDisplay::flush(I);
 	//	vpDisplay::getClick(I);
		//vpImage<vpRGBa> ItmpGet;
		//vpDisplay::getImage(I,ItmpGet);
		//vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinit.ppm");


	// 	std::cout << "class vect init ok " << std::endl;
// 		std::cout << "-- " << std::endl;
// 		for (i1 = 0 ; i1 < nbGoodSites ; i1++)
// 		{
// 			std::cout << "nOfCand "<< i1 << " : "<< numberOfCandidates[i1] << " , " ;
// 		}
// 		std::cout << std::endl << "-- " << std::endl;

		//! nbclass_corr considers only the classes with more than nbpts_min candidates.
		int nbclasses_corr = 0;
		for (int i = 0 ; i < nbclasses; i++)
		{
			if (class_size[i] > (int)(rate_nbpts_min*nbGoodSites))
			{
				nbclasses_corr ++;
			}
		}
// 		std::cout << "-- nbclasses long enough : " << nbclasses_corr << std::endl;
// 		std::cout << "-classe size : " <<endl<< class_size << std::endl;


		//!Vectors to memorise lines (result):
		std::vector<apMeLine*> mean_lines;
		mean_lines.resize(nbclasses);
		std::vector<double> mean_grad;
		mean_grad.resize(nbclasses);
		double grad;
		double mean_grad_ = 0;

		apMeLine *curr_line;
		double theta_m, rho_m;
		double co, si;
		double x,y;

		int ms,ns;
		double thetas;
		int indTheta;
		int indRho;

		int thetaSpace = 60;
		int rhoSpace = 100;
		setParameterSpace(thetaSpace,rhoSpace,I, *cam);

		int height,width;
		height=I.getHeight();
		width=I.getWidth();
		double rhol =floor(sqrt(height*height+width*width));
		std::vector<couple> couples;
        LineVote.init(thetaSpace,rhoSpace);
		int start,end;


		std::vector<apMHMeSite> vect_temp;
        double xx,yy;

		//!init means.
		for (i2 = 0 ; i2 < nbclasses ; i2++)
		{
			//mean_grad_ = 0;
			if (class_size [i2] > (int)(rate_nbpts_min*nbGoodSites))
			{
				vect_temp.clear();
				for (i1 = 0 ; i1 < nbGoodSites ; i1++)
				{
					//if (class_vect[i2][i1] != NULL)
					{
						//vect_temp.push_back(*(class_vect[i2][i1]));
					    //mean_grad_ += abs(class_vect[i2][i1]->convlt);
					ns = class_vect[i2][i1]->i;
					ms = class_vect[i2][i1]->j;
					thetas = class_vect[i2][i1]->thetaGrad+M_PI/2;
					indTheta = floor(thetas/(M_PI/thetaSpace));
					if (indTheta==thetaSpace)
						indTheta--;
					/*start = indTheta-1;
					end = indTheta+1;
					if(indTheta<1)
						start = 0;
					if(indTheta>Theta.size()-1)
						end = Theta.size();
			        for (int ii=start;ii<end;ii++)*/
			        //{
					vpPixelMeterConversion::convertPoint(*cam,ms,ns,xx,yy);
					rho = xx*cos(Theta[indTheta])+yy*sin(Theta[indTheta]) - Rho[0];
			        indRho = floor(rho/(-2*Rho[0]/rhoSpace));
			        LineVote.VoteMatrix[indTheta][indRho]=LineVote.VoteMatrix[indTheta][indRho]+1;
			        //}
				}
				}
				/*curr_line = new apMeLine;
				leastSquareLine(I,vect_temp, *cam, *curr_line, rho_m, theta_m);
				mean_lines[i2] = curr_line;*/
				//mean_grad[i2] = (mean_grad_)/nbGoodSites;
				//std::cout << " i2 "<< i2 << " mean line a " << mean_lines[i2]->a << " mean line b " << mean_lines[i2]->b << " mean line c " << mean_lines[i2]->c << std::endl;
			}
		}

		double aa,bb,cc;
		couples = LineVote.getBestVotes(nbclasses);
		for (i2 = 0 ; i2 < nbclasses ; i2++)
		{
			//curr_line = new apMeLine;
			aa = cos(Theta[couples[i2].i1]);
			bb = sin(Theta[couples[i2].i1]);
			cc = -Rho[couples[i2].j1];
			curr_line->InitLine(aa,bb,cc);
			//vpFeatureDisplay::displayLine(Rho[couples[i2].j1],Theta[couples[i2].i1], *cam, I, vpColor::yellow, 2) ;
			//std::cout << " aa "<< Rho[couples[i2].j1] << " bb "<< Theta[couples[i2].i1] << " cc "<< cc << std::endl;
			mean_lines[i2] = curr_line;
		}
		//std::cout << " votematrix "<< couples[0].vote << std::endl;
	// 	std::cout << "mean_lines init ok " << std::endl;
	// 	double t1 = vpTime::measureTimeMs();
	// 	vpDisplay::flush(I);
	// 	vpDisplay::getClick(I);
	// 	cout << "Time init : " << t1-t0 <<endl;


		////!display classification
		//std::cout << "K-mean init "<<std::endl;
 	//	col = vpColor::black;
 	//	vpDisplay::display(I);
 	//	for(unsigned int i(0);i<nbclasses;++i){
 	//		//!Display line
 	//		col = vpColor::black;
		//	/*if (class_size [i] > (int)(rate_nbpts_min*nbGoodSites))*/
 	//			mean_lines[i].display(I,col,2);
 	//		vpDisplay::flush(I);
 	//
 	//		//!Display points
 	//		if (i == 0)
 	//			col = vpColor::green;
 	//		else if (i == 1)
 	//			col = vpColor::blue;
 	//		else if (i == 2)
 	//			col = vpColor::red;
		//	else if (i == 3)
 	//			col = vpColor::black;
 	//		for(i1=0;i1 < nbGoodSites; i1++)
 	//		{
 	//			if (class_vect[i][i1] != NULL)
 	//				class_vect[i][i1]->display(I,col,7);
 	//		}
 	//	}
 	//	vpDisplay::flush(I);
 	//	vpDisplay::getClick(I);
		//
		//vpDisplay::getImage(I,ItmpGet);
		//vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinit2.ppm");

	//!*************************************************************************
	//!-------------------------------LOOP--------------------------------------
	//!*************************************************************************
		std::vector< std::vector<apMHMeSite> >::iterator it1;
		std::vector<apMHMeSite>::iterator it2;

		//std::cout  << " 0 loop mean line a " << mean_lines[0]->a << " mean line b " << mean_lines[0]->b << " mean line c " << mean_lines[0]->c << std::endl;

		int nbchange=1 ;
		bool change = false;
		int iteration=0 ;
		int classeMin = 0;
		double dmin = 300;
	// 	double dminAllCand = 300;
	// 	int i2MinAllCand;
	// 	int classMinAllCand;
		double d = 0;
		vpMatrix distances (nbclasses,nbclasses);
		vpColVector classmin(nbclasses);
		vpColVector testdiff(nbclasses);
		bool alldiff = true;
		std::vector<apMHMeSite*> vectCand;
		vectCand.resize(nbclasses);
		//apMHMeSite* site0;
		double dcand;

		double dgrad=0;
		double wgrad = 1/2;

// 		cout << "Start loop Kmean" << endl;
		//while((nbchange!=0)&&(iter < nbIterMax))
		//{
			iter++;
			double t2 = vpTime::measureTimeMs();
	// 		cout << "*** " <<iteration++<<" ***" << endl ;
			nbchange  = 0;
			mean_grad_ = 0;

			i1 = i2 = 0;
			//loop on the points
			for (i1 = 0 ; i1 < nbGoodSites ; i1++)
			{
	// 			double t4 = vpTime::measureTimeMs();
				i2 = 0;
				change = false;
				classmin = -1;
				testdiff = 0;
				distances = -1;
				alldiff = true;
// 				cout << "%%-- i1 : " <<i1<<" ----" << endl ;

				//!For each candidate, compute its distance to each valid line and make class permutations
				for (i2 = 0 ; i2 < nbclasses; i2 ++)//Parcours des candidats
				{
					//if(class_vect[i2][i1] != NULL)//case non vide
					{
 						//cout << "-- i2 : " <<i2<<" --" << endl ;
						Pk = *(class_vect[i2][i1]);
						/*site0 = new apMHMeSite;
						*site0 = *class_vect[i2][i1];
						vectCand[i2] = site0;*/
						vectCand[i2] = class_vect[i2][i1];//memorise the pointer to the site
						/*apMHMeSite Pk00 = *(class_vect[i2][i1]);
													if(Pk00.suppress == 1)
													{
																	      ip.set_i( Pk00.i );
																	      ip.set_j( Pk00.j);
																	      vpDisplay::displayCross(I, ip, 2, vpColor::white) ; // Contrast
																	    }
																	    else if(Pk00.suppress == 2) {
																	      ip.set_i( Pk00.i );
																	      ip.set_j( Pk00.j);
																	      vpDisplay::displayCross(I, ip, 2,vpColor::blue) ; // Threshold
																	    }
																	    else if(Pk00.suppress == 3) {
																	      ip.set_i( Pk00.i );
																	      ip.set_j( Pk00.j);
																	      vpDisplay::displayCross(I, ip, 3, vpColor::green) ; // M-estimator
																	    }
																	    else if(Pk00.suppress == 0) {
																	      ip.set_i( Pk00.i );
																	      ip.set_j( Pk00.j);
																		  {
																	      vpDisplay::displayCross(I, ip, 2, vpColor::blue) ; // OK
																		  }
																	    }*/

						classeMin = 0;
						dmin = 300;

						//boucle sur les classes
						//!Compute distance from the point to every good line
						for (int j = 0; j < nbclasses ; j++)
						{
							if (class_size[j] > (int)(rate_nbpts_min*nbGoodSites))//!We consider only the valid lines, computed with more than nbpts_min points.
							{
								co = cos(mean_lines[j]->getTheta());
								si = sin(mean_lines[j]->getTheta());
								vpPixelMeterConversion::convertPoint(*cam,Pk.jfloat,Pk.ifloat,x,y);
								//std::cout  <<" j "<< j << " loop mean line a " << mean_lines[j]->a << " mean line b " << mean_lines[j]->b << " mean line c " << mean_lines[j]->c << std::endl;
								//!Compute distance from the point to the line j
								d = fabs(mean_lines[j]->getRho()-(x*co+y*si));
								//dgrad = abs(abs(Pk.convlt)/mean_grad[j]);
								//std::cout << " convlt " << d << " dgrad " << dgrad << std::endl;
								//d = d + wgrad*dgrad;
								distances[i2][j] = d;
								if (d<dmin)
								{
									dmin = d;
									classeMin = j;
								}
							}
						}
						classmin[i2] = classeMin;//memorise the min class
						if (i2 != classeMin)
							change = true;


						if (testdiff[classeMin] != 0)
							alldiff = false;
						testdiff[classeMin]++;
					}
				}


				//std::cout  << " 0 loop mean line a " << mean_lines[0]->a << " mean line b " << mean_lines[0]->b << " mean line c " << mean_lines[0]->c << std::endl;
				//getchar();

// 				cout << "classemin  : "<<endl <<classmin << endl ;
				//!TODO Changements de classe !

				if ((alldiff)&&(change))
				{
					try
					{
						int a =  2;
						if (nbclasses_corr < numberOfCandidates[i1])
						 throw a;
					}
					catch(...)
					{
						cout << "############### error in find lines !#################" << endl;
						cout << "%%-- i1 : " <<i1<<" ----" << endl ;
						cout << "Test Diff : " <<endl << testdiff << endl;
						cout << "Distance : " <<endl << distances << endl;
						cout<<endl;
						cout << "##################################################" << endl;
					}
	 				cout << "-- Try change... --" << endl ;
					for (int k = 0; k < nbclasses ; k++)
					{
						//if (classmin[k] != -1)
						{//"non vide"
							cout << " change " << classmin[k] << endl ;
							class_vect[classmin[k]][i1] = vectCand[k];
							//class_vect[classmin[k]][i1] = class_vect[k][i1];
						}
						/*if (testdiff[k] == 0)//nouvelles classes vides
							class_vect[k][i1] = NULL;*/
					}
					nbchange++;
	// 				cout << "-- Change done ! --" << endl ;
				}
				else if (change)
				{
// 					cout << "%%%%%%% NOT ALL DIFFERENTS %%%%%%" <<endl;
// 					cout << "Distance : "<<endl <<distances << endl;
					double dmin_temp = 300;
					int idmin_temp = -1;
					for (int k = 0; k < nbclasses ; k++)
					{
						if (testdiff[k] > 1)
						{
// 							cout<< " hop "<<endl;
							for (int j = 0; j < nbclasses ; j++)
							{
								if (classmin[j]== k)
								{
									if (distances[j][k] < dmin_temp)
									{
										dmin_temp = distances[j][k];
										idmin_temp = j;
									}
								}
							}
							//!changement :
							if (k!=idmin_temp)
							{
								class_vect[k][i1] = vectCand[idmin_temp];
								class_vect[idmin_temp][i1] = vectCand[k];
								nbchange++;
							}
						}
					}
				}
	}
	// 			double t5 = vpTime::measureTimeMs();
	// 			cout << "time point i1 : " << t5-t4 << endl;
			//}

	// 		double t7 = vpTime::measureTimeMs();
	// 		cout << "time parcours points : " << t7-t2 << endl;
			//!Compute mean line
			//! and error
	// 		cout << "Compute new mean lines... " << endl ;

			if(recomputeline){
            apMeLine curr_line_;
			cout << "-- i2 : " << class_vect[2][5]->i <<" --" << endl ;
			for (i2 = 0 ; i2 < nbclasses ; i2++)
			{
				//mean_grad_ = 0;
				//if (class_size [i2] > (int)(rate_nbpts_min*nbGoodSites))
				{
					vect_temp.resize(0);
					for (i1 = 0 ; i1 < nbGoodSites ; i1++)
					{
						if (class_vect[i2][i1] != NULL)
							{vect_temp.push_back(*(class_vect[i2][i1]));

	                   /* apMHMeSite Pk00 = *(class_vect[i2][i1]);
							if(Pk00.suppress == 1)
							{
											      ip.set_i( Pk00.i );
											      ip.set_j( Pk00.j);
											      vpDisplay::displayCross(I, ip, 2, vpColor::white) ; // Contrast
											    }
											    else if(Pk00.suppress == 2) {
											      ip.set_i( Pk00.i );
											      ip.set_j( Pk00.j);
											      vpDisplay::displayCross(I, ip, 2,vpColor::blue) ; // Threshold
											    }
											    else if(Pk00.suppress == 3) {
											      ip.set_i( Pk00.i );
											      ip.set_j( Pk00.j);
											      vpDisplay::displayCross(I, ip, 3, vpColor::green) ; // M-estimator
											    }
											    else if(Pk00.suppress == 0) {
											      ip.set_i( Pk00.i );
											      ip.set_j( Pk00.j);
												  {
											      vpDisplay::displayCross(I, ip, 2, vpColor::blue) ; // OK
												  }
											    }*/
							}
						    //mean_grad_ += abs(class_vect[i2][i1]->convlt);
					}
					//curr_line_ = new apMeLine;
					err[i2] = leastSquareLine(I,vect_temp, *cam, curr_line_, rho_m, theta_m);
// 					cout << "err[i2] " << err[i2] << endl;
					if (err[i2] > err_max)
						err_max = err[i2];
					if (err[i2] < err_min)
						err_min = err[i2];
					*(mean_lines[i2]) = curr_line_;
					//mean_grad[i2] = mean_grad_/nbGoodSites;
				}
			}
		}

			//std::cout  << " 1 loop mean line a " << mean_lines[0]->a << " mean line b " << mean_lines[0]->b << " mean line c " << mean_lines[0]->c << std::endl;

	// 		cout<<endl;

	// 		double t8 = vpTime::measureTimeMs();
	// 		cout << "time compute new mean lines " << t8-t7 << endl;

			////!display classification
			//std::cout << "K-mean step "<<std::endl;
	 	//	col = vpColor::black;
	 	//	vpDisplay::display(I);
	 	//	for(unsigned int i(0);i<nbclasses;++i){
	 	//		//!Display line
	 	//		col = vpColor::black;
	 	//		mean_lines[i].display(I,col,2);
	 	//		vpDisplay::flush(I);
	 	//
	 	//		//!Display points
	 	//		if (i == 0)
	 	//			col = vpColor::green;
	 	//		else if (i == 1)
	 	//			col = vpColor::blue;
	 	//		else if (i == 2)
	 	//			col = vpColor::red;
			//	else if (i == 3)
 		//			col = vpColor::black;
	 	//		for(i1=0;i1 < nbGoodSites; i1++)
	 	//		{
	 	//			if (class_vect[i][i1] != NULL)
	 	//				class_vect[i][i1]->display(I,col,7);
	 	//		}
	 	//	}
	 	//	vpDisplay::flush(I);
	 	//	vpDisplay::getClick(I);
			//vpDisplay::getImage(I,ItmpGet);
			//if( iter ==1)
			//	vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinitstep1.ppm");
			//else if( iter ==2)
			//	vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinitstep2.ppm");
			//else if( iter ==3)
			//	vpImageIo::writePPM(ItmpGet, "D:/cteulier/Images/Iinitstep3.ppm");
	// 		double t3 = vpTime::measureTimeMs();
	// 		cout << "Loop time : " << t3-t2 << endl;
		//}//end loop
// 		cout << "End loop Kmean" << endl;
// 		cout << "error " <<endl << err << endl;


		//!*************************************************************************
		//!------------------------------- Results----------------------------------
		//!*************************************************************************

		weight_cumul.clear();
		double weight_cum = 0;

		double dmin_;
		double dmax_;
        double dist_;
		//!display classification and fill features_vect
// 		vpDisplay::getClick(I);

		col = vpColor::black;
// 		vpDisplay::display(I);
		points_vect.resize(nbclasses_corr);

		int j = 0;
		for(int i = 0;i<nbclasses;++i){
			//if (class_size [i] > (int)(rate_nbpts_min*nbGoodSites))
			{
				apMeLine line_tmp ;
				//line_tmp =  this->line;
				line_tmp.a = mean_lines[i]->a;
				line_tmp.b = mean_lines[i]->b;
				line_tmp.c = mean_lines[i]->c;
			    line_tmp.display(I,vpColor::black);

			    /*if(i==2)
			    vpFeatureDisplay::displayLine(mean_lines[i]->getRho(),mean_lines[i]->getTheta(), *cam, I, vpColor::purple, 1) ;
			    if(i==1)
			    vpFeatureDisplay::displayLine(mean_lines[i]->getRho(),mean_lines[i]->getTheta(), *cam, I, vpColor::yellow, 1) ;
			    if(i==0)
			    vpFeatureDisplay::displayLine(mean_lines[i]->getRho(),mean_lines[i]->getTheta(), *cam, I, vpColor::blue, 1) ;*/
				//rho ??
				//line_tmp.cam = *cam;
				//std::cout << " line a " << line_tmp.a << " line b " << line_tmp.b << " line c " << line_tmp.c << std::endl;

				//!Display points and fill vectors
				//if (i == 0)
				//	col = vpColor::green;
				//else if (i == 1)
				//	col = vpColor::blue;
				//else if (i == 2)
				//	col = vpColor::red;
			    dmin_ = 300;
			    dmax_ = 0;

				for(i1=0;i1 < nbGoodSites; i1++)
				{
					//if (class_vect[i][i1] != NULL)
					{

						//if (display_mode == mbtCadModel::ALL) class_vect[i][i1]->display(I,col,5);
						Pk = *(class_vect[i][i1]);
						co = cos(mean_lines[i]->getTheta());
						si = sin(mean_lines[i]->getTheta());
						vpPixelMeterConversion::convertPoint(*cam,Pk.jfloat,Pk.ifloat,x,y);
						dcand =fabs(mean_lines[i]->getRho()-(x*co+y*si));
						class_vect[i][i1]->setWeight(dcand);
						if (dcand < dmin_)
							dmin_ = dcand;
						if (dcand > dmax_)
							dmax_ = dcand;

					}
				}

				for(i1=0;i1 < nbGoodSites; i1++)
				{
					//if (class_vect[i][i1] != NULL)
					{

						dist_ = class_vect[i][i1]->getWeight();
						class_vect[i][i1]->setWeight((dist_-dmin_)*(dist_-dmin_)/((dmax_-dmin_)*(dmax_-dmin_)));
						points_vect[j].push_back(*class_vect[i][i1]);
						line_tmp.list.push_back(*class_vect[i][i1]);
						//cout << "i0 : " << class_vect[i][i1]->i << " j0 : " << class_vect[i][i1]->j << endl;
					}
				}


				//!Weight :
				double wght;
				/*if (err_min > 0.7)
					wght = 0.1;
				else*/
					if (err_min == err_max )
					{
						wght = exp(-(err[i])*(err[i])/0.02);
						//std::cout << "wght : " << wght << std::endl;
					}

				else
					wght = exp(-(err[i]-err_min)*(err[i]-err_min)/((err_max-err_min)*(err_max-err_min)));
// 					wght = exp(-(err[i])*(err[i])/0.02);
// 				double wght = err.euclideanNorm()/(float)err.getHeight();
				if (wght <= 0.00001) wght = 0.00001;

				std::cout << " weight line " << i << " : " << wght << std::endl;

				weights.push_back(wght);//!may need to be adjusted ?
// 				cout << "err["<< i <<"] = " <<fabs(err[i]) << endl;

				unsigned int colval = (int)255*(1-wght);
				weight_cum += wght;
				weight_cumul.push_back(weight_cum);
				//line_tmp.weight_cumul = weight_cumul;

// 				cout << "weight : " <<feature.weight << endl;
// 				cout << "colval : " <<colval << endl;
				//lines_vect.push_back(line_tmp);
				for(i1=0;i1 < nbGoodSites; i1++)
				{
					apMHMeSite p = *class_vect[i][i1];
					//cout << "i : " << p.i << " j : " << p.j << endl;
					if(p.suppress == 1)
					{
									      ip.set_i( p.i );
									      ip.set_j( p.j);
									      //vpDisplay::displayCross(I, ip, 2, vpColor::white) ; // Contrast
									    }
									    else if(p.suppress == 2) {
									      ip.set_i( p.i );
									      ip.set_j( p.j);
									      //vpDisplay::displayCross(I, ip, 2,vpColor::blue) ; // Threshold
									    }
									    else if(p.suppress == 3) {
									      ip.set_i( p.i );
									      ip.set_j( p.j);
									      //vpDisplay::displayCross(I, ip, 3, vpColor::green) ; // M-estimator
									    }
									    else if(p.suppress == 0) {
									      ip.set_i( p.i );
									      ip.set_j( p.j);
										  if(i == 0)
										  {
									      vpDisplay::displayCross(I, ip, 2, vpColor::blue) ; // OK
										  }
										  if(i == 1)
										  {
									      vpDisplay::displayCross(I, ip, 2, vpColor::yellow) ; // OK
										  }
										  if(i == 2)
										  {
									      vpDisplay::displayCross(I, ip, 2, vpColor::purple) ; // OK
										  }
									    }

				}


				//lines_vect.push_back(line_tmp);


				  /*line_tmp.list.front();

				  while (!line_tmp.list.outside())
				  {
				    vpMeSite p = line_tmp.list.value() ;

				    if(p.suppress == 1) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
				      //vpDisplay::displayCross(I, ip, 2, vpColor::white) ; // Contrast
				    }
				    else if(p.suppress == 2) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
				      vpDisplay::displayCross(I, ip, 2,vpColor::blue) ; // Threshold
				    }
				    else if(p.suppress == 3) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
				      //vpDisplay::displayCross(I, ip, 3, vpColor::green) ; // M-estimator
				    }
				    else if(p.suppress == 0) {
				      ip.set_i( p.i );
				      ip.set_j( p.j);
					  if(i == 0)
					  {
				      vpDisplay::displayCross(I, ip, 2, vpColor::blue) ; // OK
					  }
					  if(i == 1)
					  {
				      vpDisplay::displayCross(I, ip, 2, vpColor::yellow) ; // OK
					  }
					  if(i == 2)
					  {
				      vpDisplay::displayCross(I, ip, 2, vpColor::purple) ; // OK
					  }
				    }
				    line_tmp.list.next() ;
				  }*/


				//!Display line
				//Display line according to its weight


				//col.setColor(colval,colval,colval);

				 // Draw a red rectangle in the display overlay (foreground)
// 				((vpDisplayX*)(I.display))->displayLine( 100, 10, 80, 400, col, 2);

				//line_tmp.display(I,col);

				//vpDisplay::flush(I);
 			//	vpDisplay::getClick(I);

// 				if (display_mode == mbtCadModel::ALL)
// 				{
 //// 					vpRGBa greyCol(colval);
 //					col = vpColor::black;
 //// 					mean_lines[i].display(I,col);
 //					mean_lines[i].display(I,col);
 //					vpDisplay::flush(I);
// 				}


				j++;
			}
		}

// 		cout << "err_min : " << err_min << endl;
// 		cout << "err_max : " << err_max << endl;
		//vpDisplay::flush(I);
		//!TEST
// 		vpHomogeneousMatrix cMo;
// 		features_vect[0].line.Project(cMo);
		//!
// 		vpDisplay::getClick(I);

		//double t2 = vpTime::measureTimeMs();
		//std::cout <<"******* find lines time: " << t2-t1<<std::endl;

// 		DisplayListAll(I,vpColor::red,3);
		//if (DEBUG_LEVEL1)
			std::cout << "end vpMeTracker::findLinesByKmean() " << std::endl;
		return j;
	}
	else
	{
		//double t2 = vpTime::measureTimeMs();
		//std::cout <<"******* find lines time: " << t2-t1<<std::endl;
		//no candidate for that feature (not enough points)
		//if (DEBUG_LEVEL1)
			std::cout << "end vpMeTracker::findLinesByKmean() (not enough points)" << std::endl ;
		return 0;
	}


}




/*!
  Update the moving edges parameters after the virtual visual servoing.

  \param  I : The image.
  \param  rho : The \f$\rho\f$ parameter used in the line's polar equation.
  \param  theta : The \f$\theta\f$ parameter used in the line's polar equation.
*/
void
apMHMeLine::updateParameters(const vpImage<unsigned char> &I, double rho, double theta)
{
  this->rho = rho;
  this->theta = theta;
  a = cos(theta);
  b = sin(theta);
  c = -rho;
  // recherche de point aux extremite de la droites
  // dans le cas d'un glissement
  suppressPoints(I);
  seekExtremities(I);
  suppressPoints(I);
  setExtremities();
  //reechantillonage si necessaire
  reSample(I);

  // remet a jour l'angle delta pour chaque  point de la liste
  updateDelta();
}


/*!
  Update the moving edges parameters after the virtual visual servoing.

  \param I : The image.
  \param ip1 : The first extremity of the line.
  \param ip2 : The second extremity of the line.
  \param rho : The \f$\rho\f$ parameter used in the line's polar equation.
  \param theta : The \f$\theta\f$ parameter used in the line's polar equation.
*/
void
apMHMeLine::updateParameters(const vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2, double rho, double theta)
{
  this->rho = rho;
  this->theta = theta;
  a = cos(theta);
  b = sin(theta);
  c = -rho;
  // recherche de point aux extremite de la droites
  // dans le cas d'un glissement
  suppressPoints(I);
  seekExtremities(I);
  suppressPoints(I);
  setExtremities();
  //reechantillonage si necessaire
  reSample(I,ip1,ip2);

  // remet a jour l'angle delta pour chaque  point de la liste
  updateDelta();
}


/*!
  Seek in the list of available points the two extremities of the line.
*/
void
apMHMeLine::setExtremities()
{
  double imin = +1e6 ;
  double jmin = +1e6;
  double imax = -1 ;
  double jmax = -1 ;

  // Loop through list of sites to track
  for (int l =0; l<list.size(); l++)
  {
    apMHMeSite s = list[l] ;//current reference pixel
    if (s.ifloat < imin)
    {
      imin = s.ifloat ;
      jmin = s.jfloat ;
    }

    if (s.ifloat > imax)
    {
      imax = s.ifloat ;
      jmax = s.jfloat ;
    }
  }

  if (list.size() != 0)
  {
    PExt[0].ifloat = imin ;
    PExt[0].jfloat = jmin ;
    PExt[1].ifloat = imax ;
    PExt[1].jfloat = jmax ;
  }

  if (fabs(imin-imax) < 25)
  {
	for (int l =0; l<list.size(); l++)
	{
      apMHMeSite s = list[l] ;//current reference pixel
      if (s.jfloat < jmin)
      {
	imin = s.ifloat ;
	jmin = s.jfloat ;
      }

      if (s.jfloat > jmax)
      {
	imax = s.ifloat ;
	jmax = s.jfloat ;
      }
    }

    if (list.size() != 0)
    {
      PExt[0].ifloat = imin ;
      PExt[0].jfloat = jmin ;
      PExt[1].ifloat = imax ;
      PExt[1].jfloat = jmax ;
    }
    bubbleSortJ();
  }

  else
    bubbleSortI();
}


void
apMHMeLine::bubbleSortI()
{
  int nbElmt = list.size();
  for (int pass = 1; pass < nbElmt; pass++)
  {
    for (int i=0; i < nbElmt-pass; i++)
    {
      apMHMeSite s1 = list[i] ;
      apMHMeSite s2 = list[i+1] ;
      if (s1.ifloat > s2.ifloat)
        {
        list[i] = s2;
        list[i+1] = s1;
        i--;
        }
    }
  }
}


void
apMHMeLine::bubbleSortJ()
{
  int nbElmt = list.size();
  for(int pass=1; pass < nbElmt; pass++)
  {
    for (int i=0; i < nbElmt-pass; i++)
    {
        apMHMeSite s1 = list[i] ;
        apMHMeSite s2 = list[i+1] ;
      if (s1.jfloat > s2.jfloat)
      {
      list[i] = s2;
      list[i+1] = s1;
      i--;
      }
    }
  }
}


void
apMHMeLine::findSignal(const vpImage<unsigned char>& I, const vpMe *me, double *conv)
{
  vpImagePoint itest(PExt[0].ifloat+(PExt[1].ifloat-PExt[0].ifloat)/2, PExt[0].jfloat+(PExt[1].jfloat-PExt[0].jfloat)/2);

  vpMeSite pix ; //= list.value();
  pix.init(itest.get_i(), itest.get_j(), delta, 0, sign);

  vpMeSite  *list_query_pixels;
//  double  convolution = 0;
  int range  = me->range;

  list_query_pixels = pix.getQueryList(I, range);

  vpDisplay::displayCross(I,itest,5,vpColor::cyan,3);
  vpDisplay::displayLine(I,vpImagePoint(list_query_pixels[0].ifloat,list_query_pixels[0].jfloat),vpImagePoint(list_query_pixels[2*range].ifloat,list_query_pixels[2*range].jfloat),vpColor::cyan);
  vpDisplay::displayCross(I,vpImagePoint(list_query_pixels[0].ifloat,list_query_pixels[0].jfloat),5,vpColor::orange,3);

  for(int n = 0 ; n < 2 * range + 1 ; n++)
  {
    conv[n] = list_query_pixels[n].convolution(I, me);
  }
  delete [] list_query_pixels;
}
