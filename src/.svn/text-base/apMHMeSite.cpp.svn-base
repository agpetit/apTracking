/*
 * apMHMeSite.cpp
 *
 *  Created on: Jul 9, 2012
 *      Author: agpetit
 */
#include <visp/vpImageIo.h>
#include "apMHMeSite.h"

#ifndef DOXYGEN_SHOULD_SKIP_THIS
static
bool horsImage( int i , int j , int half , int rows , int cols)
{
  return((i < half + 1) || ( i > (rows - half - 3) )||(j < half + 1) || (j > (cols - half - 3) )) ;
}
#endif

void
apMHMeSite::init()
{
  // Site components
  alpha =  0.0 ;
  convlt = 0.0 ;
  suppress = 0;
  weight=-1;

  selectDisplay = NONE ;

  // Pixel components
  i = 0 ;
  j = 0 ;
  v = 0 ;
  ifloat =i ;
  jfloat = j ;
  i_1 = 0 ;
  j_1 = 0 ;

  mask_sign =1 ;
  normGradient = 0;

  //std::cout<<profile<<std::endl;
}


apMHMeSite::apMHMeSite(double ip, double jp)
{
  init() ;

  selectDisplay = NONE ;
  i = vpMath::round(ip) ;
  j = vpMath::round(jp) ;
  ifloat = ip ;
  jfloat = jp ;
	candidateList = NULL;
	numCandidates = 3;
	thetaGrad = 0;
}


apMHMeSite::apMHMeSite() {
	// TODO Auto-generated constructor stub
	init();
	candidateList = NULL;
	numCandidates = 3;
	thetaGrad = 0;

}

apMHMeSite::apMHMeSite(const apMHMeSite & m) : candidateList(NULL)
{
	i = m.i ;
	j = m.j ;
	i_1 = m.i_1 ;
	j_1 = m.j_1 ;
	alpha = m.alpha ;
	v = m.v ;
	ifloat = m.ifloat ;
	jfloat = m.jfloat ;
	convlt = m.convlt ;
	suppress = m.suppress;
	weight = m.weight;
	normGradient = m.normGradient;

	numCandidates = m.numCandidates;
	selectDisplay = m.selectDisplay ;
	mask_sign = m.mask_sign ;

	thetaGrad = m.thetaGrad;
	weight_s = m.weight_s;
	  *this = m;
	//candidateList = m.candidateList;
	//candidateVect = m.candidateVect;

}

// More an Update than init
// For points in meter form (to avoid approximations)
void
apMHMeSite::init(double ip, double jp, double alphap)
{
  // Note: keep old value of convlt, suppress and contrast
  selectDisplay = NONE ;

  ifloat = ip;
  i= vpMath::round(ip);
  jfloat = jp ;
  j = vpMath::round(jp);
  alpha = alphap ;
  mask_sign =1 ;

  v = 0 ;
  i_1 = 0 ;
  j_1 = 0 ;

}

// initialise with convolution
void
apMHMeSite::init(double ip, double jp, double alphap, double convltp)
{
  selectDisplay = NONE ;
  ifloat = ip ;
  i= (int)ip ;
  jfloat = jp ;
  j =(int)jp  ;
  alpha = alphap ;
  convlt = convltp;
  mask_sign =1 ;

  v = 0 ;
  i_1 = 0 ;
  j_1 = 0 ;

}
// initialise with convolution and sign
void
apMHMeSite::init(double ip, double jp, double alphap, double convltp, int sign)
{
  selectDisplay = NONE ;
  ifloat = ip ;
  i= (int)ip ;
  jfloat = jp ;
  j =(int)jp  ;
  alpha = alphap ;
  convlt = convltp;
  mask_sign = sign ;

  v = 0 ;
  i_1 = 0 ;
  j_1 = 0 ;
}


apMHMeSite &apMHMeSite::operator=(const apMHMeSite &m)
{
	i = m.i ;
	j = m.j ;
	i_1 = m.i_1 ;
	j_1 = m.j_1 ;
	alpha = m.alpha ;
	v = m.v ;
	ifloat = m.ifloat ;
	jfloat = m.jfloat ;
	convlt = m.convlt ;
	suppress = m.suppress;
	weight = m.weight;
	normGradient = m.normGradient;

	numCandidates = m.numCandidates;

	selectDisplay = m.selectDisplay ;
	mask_sign = m.mask_sign ;

	thetaGrad = m.thetaGrad;
	weight_s = m.weight_s;
	//candidateList = m.candidateList;
	//candidateVect = m.candidateVect;

	return *this ;
}

apMHMeSite::~apMHMeSite() {
	// TODO Auto-generated destructor stub
	if (candidateList != NULL)
		delete [] candidateList;
}


apMHMeSite*
apMHMeSite::getQueryList(const vpImage<unsigned char> &I, const int range)
{

  int   k ;

  int n;
  double ii , jj ;
  apMHMeSite *list_query_pixels ;
  list_query_pixels =  NULL ;

  // Size of query list includes the point on the line
  list_query_pixels = new apMHMeSite[2 * range + 1] ;

  // range : +/- the range within which the pixel's
  //correspondent will be sought

  double salpha = sin(alpha);
  double calpha = cos(alpha);
  n = 0 ;
  vpImagePoint ip;

  for(k = -range ; k <= range ; k++)
  {
    ii = (ifloat+k*salpha);
    jj = (jfloat+k*calpha);

	// Display
    if    ((selectDisplay==RANGE_RESULT)||(selectDisplay==RANGE)) {
      ip.set_i( ii );
      ip.set_j( jj );
      vpDisplay::displayCross(I, ip, 1, vpColor::yellow) ;
    }

    // Copy parent's convolution
    apMHMeSite pel ;
    pel.init(ii, jj, alpha, convlt,mask_sign) ;
    pel.setDisplay(selectDisplay) ;// Display

	// Add site to the query list
    list_query_pixels[n] = pel ;
    n++ ;
  }

  return(list_query_pixels) ;
}

// ===================================================================
/*!
 * get the sign (according to the difference of values of the intensities of the extremities).
 * \pre : ifloat, jfloat, and the direction of the normal (alpha) have to be set.
 * \param I : Image in which the sign is computed.
 * \param range :  +/- the range within which the pixel's correspondent is sought
 * \post : mask_sign is computed
 */
// ===================================================================
void
apMHMeSite::getSign(const vpImage<unsigned char> &I, const int range)
{

  int   k ;

  double salpha = sin(alpha);
  double calpha = cos(alpha);

	//First extremity
  k = -range ;
  int i1 = vpMath::round(ifloat+k*salpha);
  int j1 = vpMath::round(jfloat+k*calpha);

	//Second extremity
  k = range ;
  int i2 = vpMath::round(ifloat+k*salpha);
  int j2 = vpMath::round(jfloat+k*calpha);

  if (I[i1][j1] > I[i2][j2]) mask_sign = 1 ; else mask_sign = -1 ;
}


// Specific function for ME
double
apMHMeSite::convolution(const vpImage<unsigned char>&I, const  vpMe *me)
{

  int half, index_mask ;
  double conv = 0.0 ;
  half = (me->mask_size - 1) >> 1 ;

  if(horsImage( i , j , half + me->strip , I.getHeight(), I.getWidth()))
  {
    conv = 0.0 ;
    i = 0 ; j = 0 ;
  }
  else
  {
    // Calculate tangent angle from normal
    double theta  = alpha+M_PI/2;
    // Move tangent angle to within 0->M_PI for a positive
    // mask index
    while (theta<0) theta += M_PI;
    while (theta>M_PI) theta -= M_PI;

    // Convert radians to degrees
    int thetadeg = vpMath::round(theta * 180 / M_PI) ;

    if(abs(thetadeg) == 180 )
    {
      thetadeg= 0 ;
    }

    index_mask = (int)(thetadeg/(double)me->anglestep);

    int ihalf = i-half ;
    int jhalf = j-half ;
    int ihalfa ;
    int a ;
    int b ;
    for( a = 0 ; a < me->mask_size ; a++ )
    {
      ihalfa = ihalf+a ;
      for( b = 0 ; b < me->mask_size ; b++ )
      {
	conv += mask_sign* me->mask[index_mask][a][b] *
	  //	  I(i-half+a,j-half+b) ;
	  I(ihalfa,jhalf+b) ;
      }
    }

  }

  return(conv) ;
}

apMHMeSite*
apMHMeSite::getQueryListMH(const vpImage<unsigned char> &I, const int range)
{

  int   k ;

  int n;
  double ii , jj ;
  apMHMeSite *list_query_pixels ;
  list_query_pixels =  NULL ;

  // Size of query list includes the point on the line
  list_query_pixels = new apMHMeSite[2 * range + 1] ;

  // range : +/- the range within which the pixel's
  //correspondent will be sought

  double salpha = sin(alpha);
  double calpha = cos(alpha);
  n = 0 ;
  vpImagePoint ip;

  for(k = -range ; k <= range ; k++)
  {
    ii = (ifloat+k*salpha);
    jj = (jfloat+k*calpha);

	// Display
    if    ((selectDisplay==RANGE_RESULT)||(selectDisplay==RANGE)) {
      ip.set_i( ii );
      ip.set_j( jj );
      vpDisplay::displayCross(I, ip, 1, vpColor::yellow) ;
    }

    // Copy parent's convolution
    apMHMeSite pel ;
    pel.init(ii, jj, alpha, convlt,mask_sign) ;
    pel.setDisplay(selectDisplay) ;// Display

	// Add site to the query list
    list_query_pixels[n] = pel ;
    n++ ;
  }

  return(list_query_pixels) ;
}

void
apMHMeSite::track0(const vpImage<unsigned char>& I,
		const vpMe *me,
		const bool test_contraste)
{
//   vpMeSite  *list_query_pixels ;
//   int  max_rank =0 ;
//   int max_rank1=0 ;
//   int max_rank2 = 0;
//   double  convolution = 0 ;
//   double  max_convolution = 0 ;
//   double max = 0 ;
//   double contraste = 0;
//   //  vpDisplay::display(I) ;
//   //  vpERROR_TRACE("getclcik %d",me->range) ;
//   //  vpDisplay::getClick(I) ;
//
//   // range = +/- range of pixels within which the correspondent
//   // of the current pixel will be sought
//   int range  = me->range ;
//
//   //  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
//   list_query_pixels = getQueryList(I, range) ;
//
//   double  contraste_max = 1 + me->mu2 ;
//   double  contraste_min = 1 - me->mu1 ;
//
//   // array in which likelihood ratios will be stored
//   double  *likelihood= new double[ 2 * range + 1 ] ;
//
//   int ii_1 = i ;
//   int jj_1 = j ;
//   i_1 = i ;
//   j_1 = j ;
//   double threshold;
//   threshold = me->threshold ;
//
//   //    std::cout <<"---------------------"<<std::endl ;
//   for(int n = 0 ; n < 2 * range + 1 ; n++)
//     {
//
//       //   convolution results
//       convolution = list_query_pixels[n].convolution(I, me) ;
//
//       // luminance ratio of reference pixel to potential correspondent pixel
//       // the luminance must be similar, hence the ratio value should
//       // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
//       if( test_contraste )
// 	{
// 	  // Include this to eliminate temporal calculation
// 	  if (convlt==0)
// 	    {
// 	      std::cout << "vpMeSite::track : Division by zero  convlt = 0" << std::endl ;
// 	      delete []list_query_pixels ;
// 	      delete []likelihood;
// 	      throw(vpTrackingException(vpTrackingException::initializationError,
// 					"Division by zero")) ;
// 	    }
//
// // 	  contraste = fabs(convolution / convlt) ;
// 	  contraste = convolution / convlt ;
// 	  // likelihood ratios
// 	  if((contraste > contraste_min) && (contraste < contraste_max))
// 	    likelihood[n] = fabs(convolution + convlt ) ;
// 	  else
// 	    likelihood[n] = 0 ;
// 	}
//       else
// 	likelihood[n] = fabs(2*convolution) ;
//
//
//       // establishment of the maximal likelihood ratios's  rank
//       // in the array, the value of the likelihood ratio can now be
//     // referenced by its rank in the array
//     if (likelihood[n] > max)
//     {
//       max_convolution= convolution;
//       max = likelihood[n] ;
//       max_rank = n ;
//       max_rank2 = max_rank1;
//       max_rank1 = max_rank;
//     }
//
//   }
//
//   // test on the likelihood threshold if threshold==-1 then
//   // the me->threshold is  selected
//
//   vpImagePoint ip;
//
//   //  if (test_contrast)
//   if(max > threshold)
//     {
//       if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
// 	{
// 	  ip.set_i( list_query_pixels[max_rank].i );
// 	  ip.set_j( list_query_pixels[max_rank].j );
// 	  vpDisplay::displayPoint(I, ip, vpColor::red);
// 	}
//
//       *this = list_query_pixels[max_rank] ;//The vpMeSite is replaced by the vpMeSite of max likelihood
//       normGradient =  vpMath::sqr(max_convolution);
//
//       convlt = max_convolution;
//       i_1 = ii_1; //list_query_pixels[max_rank].i ;
//       j_1 = jj_1; //list_query_pixels[max_rank].j ;
//       delete []list_query_pixels ;
//       delete []likelihood;
//     }
//   else //none of the query sites is better than the threshold
//     {
//       if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
// 	{
// 	  ip.set_i( list_query_pixels[max_rank].i );
// 	  ip.set_j( list_query_pixels[max_rank].j );
// 	  vpDisplay::displayPoint(I, ip, vpColor::green);
// 	}
//       normGradient = 0 ;
//       if(max == 0)
// 	suppress = 1; // contrast suppression
//       else
// 	suppress = 2; // threshold suppression
//
//       delete []list_query_pixels ;
//       delete []likelihood; // modif portage
//     }

  apMHMeSite  *list_query_pixels ;
  int  max_rank =-1 ;
  int max_rank1=-1 ;
  int max_rank2 = -1;
  double  convolution = 0 ;
  double  max_convolution = 0 ;
  double max = 0 ;
  double contraste = 0;
  //  vpDisplay::display(I) ;
  //  vpERROR_TRACE("getclcik %d",me->range) ;
  //  vpDisplay::getClick(I) ;

  // range = +/- range of pixels within which the correspondent
  // of the current pixel will be sought
  int range  = me->range ;

  //  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
  list_query_pixels = getQueryList(I, range) ;

  double  contraste_max = 1 + me->mu2 ;
  double  contraste_min = 1 - me->mu1 ;

  // array in which likelihood ratios will be stored
  double  *likelihood= new double[ 2 * range + 1 ] ;

  int ii_1 = i ;
  int jj_1 = j ;
  i_1 = i ;
  j_1 = j ;
  double threshold;
  threshold = me->threshold ;
  double diff = 1e6;

  //    std::cout <<"---------------------"<<std::endl ;
  for(int n = 0 ; n < 2 * range + 1 ; n++)
  {
      //   convolution results
      convolution = list_query_pixels[n].convolution(I, me) ;

      // luminance ratio of reference pixel to potential correspondent pixel
      // the luminance must be similar, hence the ratio value should
      // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
      if( test_contraste )
      {
	likelihood[n] = fabs(convolution + convlt );
	if (likelihood[n]> threshold)
	{
	  contraste = convolution / convlt;
	  if((contraste > contraste_min) && (contraste < contraste_max) && fabs(1-contraste) < diff)
	  {
	    diff = fabs(1-contraste);
	    max_convolution= convolution;
	    max = likelihood[n] ;
	    max_rank = n ;
	    max_rank2 = max_rank1;
	    max_rank1 = max_rank;
	  }
	}
      }

      else
      {
	likelihood[n] = fabs(2*convolution) ;
	if (likelihood[n] > max  && likelihood[n] > threshold)
	{
	  max_convolution= convolution;
          max = likelihood[n] ;
          max_rank = n ;
          max_rank2 = max_rank1;
          max_rank1 = max_rank;
        }
      }
  }

  // test on the likelihood threshold if threshold==-1 then
  // the me->threshold is  selected

  vpImagePoint ip;

  //  if (test_contrast)
  if(max_rank >= 0)
    {
      if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
	{
	  ip.set_i( list_query_pixels[max_rank].i );
	  ip.set_j( list_query_pixels[max_rank].j );
	  vpDisplay::displayPoint(I, ip, vpColor::red);
	}

      *this = list_query_pixels[max_rank] ;//The vpMeSite2 is replaced by the vpMeSite2 of max likelihood
      normGradient =  vpMath::sqr(max_convolution);
      //std::cout << " j " << list_query_pixels[max_rank].j << " i " << list_query_pixels[max_rank].i << std::endl;

      convlt = max_convolution;
      i_1 = ii_1; //list_query_pixels[max_rank].i ;
      j_1 = jj_1; //list_query_pixels[max_rank].j ;
      delete []list_query_pixels;
      delete []likelihood;
    }
  else //none of the query sites is better than the threshold
    {
      if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
	{
	  ip.set_i( list_query_pixels[0].i );
	  ip.set_j( list_query_pixels[0].j );
	  vpDisplay::displayPoint(I, ip, vpColor::green);
	}
      normGradient = 0 ;
      if(contraste != 0)
	suppress = 1; // contrast suppression
      else
	suppress = 2; // threshold suppression

      delete []list_query_pixels ;
      delete []likelihood; // modif portage
    }
}


void
apMHMeSite::track(const vpImage<unsigned char>& I,
		const vpMe *me,
		const bool test_contraste)
{
//   vpMeSite  *list_query_pixels ;
//   int  max_rank =0 ;
//   int max_rank1=0 ;
//   int max_rank2 = 0;
//   double  convolution = 0 ;
//   double  max_convolution = 0 ;
//   double max = 0 ;
//   double contraste = 0;
//   //  vpDisplay::display(I) ;
//   //  vpERROR_TRACE("getclcik %d",me->range) ;
//   //  vpDisplay::getClick(I) ;
//
//   // range = +/- range of pixels within which the correspondent
//   // of the current pixel will be sought
//   int range  = me->range ;
//
//   //  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
//   list_query_pixels = getQueryList(I, range) ;
//
//   double  contraste_max = 1 + me->mu2 ;
//   double  contraste_min = 1 - me->mu1 ;
//
//   // array in which likelihood ratios will be stored
//   double  *likelihood= new double[ 2 * range + 1 ] ;
//
//   int ii_1 = i ;
//   int jj_1 = j ;
//   i_1 = i ;
//   j_1 = j ;
//   double threshold;
//   threshold = me->threshold ;
//
//   //    std::cout <<"---------------------"<<std::endl ;
//   for(int n = 0 ; n < 2 * range + 1 ; n++)
//     {
//
//       //   convolution results
//       convolution = list_query_pixels[n].convolution(I, me) ;
//
//       // luminance ratio of reference pixel to potential correspondent pixel
//       // the luminance must be similar, hence the ratio value should
//       // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
//       if( test_contraste )
// 	{
// 	  // Include this to eliminate temporal calculation
// 	  if (convlt==0)
// 	    {
// 	      std::cout << "vpMeSite::track : Division by zero  convlt = 0" << std::endl ;
// 	      delete []list_query_pixels ;
// 	      delete []likelihood;
// 	      throw(vpTrackingException(vpTrackingException::initializationError,
// 					"Division by zero")) ;
// 	    }
//
// // 	  contraste = fabs(convolution / convlt) ;
// 	  contraste = convolution / convlt ;
// 	  // likelihood ratios
// 	  if((contraste > contraste_min) && (contraste < contraste_max))
// 	    likelihood[n] = fabs(convolution + convlt ) ;
// 	  else
// 	    likelihood[n] = 0 ;
// 	}
//       else
// 	likelihood[n] = fabs(2*convolution) ;
//
//
//       // establishment of the maximal likelihood ratios's  rank
//       // in the array, the value of the likelihood ratio can now be
//     // referenced by its rank in the array
//     if (likelihood[n] > max)
//     {
//       max_convolution= convolution;
//       max = likelihood[n] ;
//       max_rank = n ;
//       max_rank2 = max_rank1;
//       max_rank1 = max_rank;
//     }
//
//   }
//
//   // test on the likelihood threshold if threshold==-1 then
//   // the me->threshold is  selected
//
//   vpImagePoint ip;
//
//   //  if (test_contrast)
//   if(max > threshold)
//     {
//       if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
// 	{
// 	  ip.set_i( list_query_pixels[max_rank].i );
// 	  ip.set_j( list_query_pixels[max_rank].j );
// 	  vpDisplay::displayPoint(I, ip, vpColor::red);
// 	}
//
//       *this = list_query_pixels[max_rank] ;//The vpMeSite is replaced by the vpMeSite of max likelihood
//       normGradient =  vpMath::sqr(max_convolution);
//
//       convlt = max_convolution;
//       i_1 = ii_1; //list_query_pixels[max_rank].i ;
//       j_1 = jj_1; //list_query_pixels[max_rank].j ;
//       delete []list_query_pixels ;
//       delete []likelihood;
//     }
//   else //none of the query sites is better than the threshold
//     {
//       if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
// 	{
// 	  ip.set_i( list_query_pixels[max_rank].i );
// 	  ip.set_j( list_query_pixels[max_rank].j );
// 	  vpDisplay::displayPoint(I, ip, vpColor::green);
// 	}
//       normGradient = 0 ;
//       if(max == 0)
// 	suppress = 1; // contrast suppression
//       else
// 	suppress = 2; // threshold suppression
//
//       delete []list_query_pixels ;
//       delete []likelihood; // modif portage
//     }

  apMHMeSite  *list_query_pixels ;
  int  max_rank =-1 ;
  int max_rank1=-1 ;
  int max_rank2 = -1;
  double  convolution = 0 ;
  double  max_convolution = 0 ;
  double max = 0 ;
  double contraste = 0;
  //  vpDisplay::display(I) ;
  //  vpERROR_TRACE("getclcik %d",me->range) ;
  //  vpDisplay::getClick(I) ;

  // range = +/- range of pixels within which the correspondent
  // of the current pixel will be sought
  int range  = me->range ;

  //  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
  list_query_pixels = getQueryList(I, range) ;

  double  contraste_max = 1 + me->mu2 ;
  double  contraste_min = 1 - me->mu1 ;

  // array in which likelihood ratios will be stored
  double  *likelihood= new double[ 2 * range + 1 ] ;

  int ii_1 = i ;
  int jj_1 = j ;
  i_1 = i ;
  j_1 = j ;
  double threshold;
  threshold = me->threshold ;
  double diff = 1e6;

  //    std::cout <<"---------------------"<<std::endl ;
  for(int n = 0 ; n < 2 * range + 1 ; n++)
  {
      //   convolution results
      convolution = list_query_pixels[n].convolution(I, me) ;

      // luminance ratio of reference pixel to potential correspondent pixel
      // the luminance must be similar, hence the ratio value should
      // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
      if( test_contraste )
      {
	likelihood[n] = fabs(convolution + convlt );
	if (likelihood[n]> threshold)
	{
	  contraste = convolution / convlt;
	  if((contraste > contraste_min) && (contraste < contraste_max) && fabs(1-contraste) < diff)
	  {
	    diff = fabs(1-contraste);
	    max_convolution= convolution;
	    max = likelihood[n] ;
	    max_rank = n ;
	    max_rank2 = max_rank1;
	    max_rank1 = max_rank;
	  }
	}
      }

      else
      {
	likelihood[n] = fabs(2*convolution) ;
	if (likelihood[n] > max  && likelihood[n] > threshold)
	{
	  max_convolution= convolution;
          max = likelihood[n] ;
          max_rank = n ;
          max_rank2 = max_rank1;
          max_rank1 = max_rank;
        }
      }
  }

  // test on the likelihood threshold if threshold==-1 then
  // the me->threshold is  selected

  vpImagePoint ip;

  //  if (test_contrast)
  if(max_rank >= 0)
    {
      if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
	{
	  ip.set_i( list_query_pixels[max_rank].i );
	  ip.set_j( list_query_pixels[max_rank].j );
	  vpDisplay::displayPoint(I, ip, vpColor::red);
	}

      *this = list_query_pixels[max_rank] ;//The vpMeSite2 is replaced by the vpMeSite2 of max likelihood
      normGradient =  vpMath::sqr(max_convolution);

      convlt = max_convolution;
      i_1 = ii_1; //list_query_pixels[max_rank].i ;
      j_1 = jj_1; //list_query_pixels[max_rank].j ;
      delete []list_query_pixels;
      delete []likelihood;
    }
  else //none of the query sites is better than the threshold
    {
      if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
	{
	  ip.set_i( list_query_pixels[0].i );
	  ip.set_j( list_query_pixels[0].j );
	  vpDisplay::displayPoint(I, ip, vpColor::green);
	}
      normGradient = 0 ;
      if(contraste != 0)
	suppress = 1; // contrast suppression
      else
	suppress = 2; // threshold suppression

      delete []list_query_pixels ;
      delete []likelihood; // modif portage
    }
}

void
apMHMeSite::trackMH(const vpImage<unsigned char>& I,
		const vpMe *me,
		const bool test_contraste)
{
// 	vpDisplay::displayCross(I,i, j, 3,vpColor::blue) ;

//if (candidateList == NULL)
		candidateList = new apMHMeSite[numCandidates];

//std::cout << "numCandidates " << numCandidates << std::endl;

	apMHMeSite  *list_query_pixels ;

	int * ranks = new int[numCandidates];
	for (int k = 0 ; k < numCandidates ; k++)
		ranks[k] = 0;

	int max_rank = 0;
	double  convolution = 0 ;
	double  last_convolution = 0 ;
	double  max_convolution = 0 ;
	double max = 0 ;
	double min = 0 ;
	int min_rank = 0 ;
	double contraste = 0;
  //  vpDisplay::display(I) ;
  //  vpERROR_TRACE("getclcik %d",me->range) ;
  //  vpDisplay::getClick(I) ;

  // range = +/- range of pixels within which the correspondent
  // of the current pixel will be sought
	int range  = me->range ;

	if ((range*2 + 1 < numCandidates)&&(range>0))
	{
		std::cout << "WARNING: 2* range +1 < numCandidates. Range = " << range << " is reset to numCandidates/2 = " << numCandidates/2 << std::endl;
		range = numCandidates/2;
	}
	//  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
	list_query_pixels = getQueryList(I, range) ;

	double  contraste_max = 1 + me->mu2 ;
	double  contraste_min = 1 - me->mu1 ;

	// array in which likelihood ratios will be stored
	double  *likelihood = new double[ 2 * range + 1 ] ;

	int ii_1 = i ;//!What for?
	int jj_1 = j ;
	i_1 = i ;
	j_1 = j ;
	double threshold;
	threshold = me->threshold ;

  //    std::cout <<"---------------------"<<std::endl ;
  	//! Loop on the list of query pixels. Find the best candidates in terms of likelihood value
	bool up = false;//indicate if likelihood is increasing
	for(int n = 0 ; n < 2 * range + 1 ; n++)
	{

		//   convolution results
		last_convolution = convolution;
		convolution = list_query_pixels[n].convolution(I, me) ;

		// luminance ratio of reference pixel to potential correspondent pixel
		// the luminance must be similar, hence the ratio value should
		// lay between, for instance, 0.5 and 1.5 (parameter tolerance)

		if( test_contraste )
		{
			// Include this to eliminate temporal calculation
			if (convlt==0)
			{
				std::cout << "vpMeSite::track : Division by zero  convlt = 0" << std::endl ;
				delete []list_query_pixels ;
				delete []likelihood;
				delete []ranks;
				//throw(vpTrackingException(vpTrackingException::initializationError,
					//		"Division by zero")) ;
			}

			contraste = fabs(convolution / convlt) ;
			// likelihood ratios
			if((contraste > contraste_min) && (contraste < contraste_max))
				likelihood[n] = fabs(convolution + convlt ) ;
			else
				likelihood[n] = 0 ;
		}
		else
			likelihood[n] = fabs(2*convolution) ;

		//std::cout << " track mh " << likelihood[1]  << " " << likelihood[2] << " " << likelihood[3] << std::endl;

 		//std::cout << "convlt : " << convlt << " , Convolution : " << convolution << " , Likelihood : " << likelihood[n]<< " ncand "  << std::endl;
		//To obtain the numCandidates best candidates of the list of vpMeSite: (tested)
		if (n < numCandidates)
		{
			if (n==0)
				min = likelihood[n];
			else
			{
				if (likelihood[n] < min)
				{
					min_rank = n;
					min = likelihood[n];
				}
				if (likelihood[n] > likelihood[n-1])
					up = true;
			}

			candidateList[n] = list_query_pixels[n];
			ranks[n] = n;
			candidateList[n].convlt = convolution;
			candidateList[min_rank].normGradient =  vpMath::sqr(convolution);
		}
		else
		{
			if ((likelihood[n] < likelihood[n-1])&&(up==true))//changement de direction, n-1 était un max local
			{
				if (likelihood[n-1]>min)
				{
					candidateList[min_rank] = list_query_pixels[n-1];
					ranks[min_rank] = n-1;
					candidateList[min_rank].convlt = last_convolution;
					candidateList[min_rank].normGradient =  vpMath::sqr(last_convolution);
	// 				Compute new min of candidateList
					min = likelihood[ranks[0]];
					min_rank=0;
					for (int k = 1 ; k < numCandidates ; k++)
					{
						if (likelihood[ranks[k]] < min)
						{
							min = likelihood[ranks[k]];
							min_rank = k;
						}
					}
				}
			}
			up = (likelihood[n] >= likelihood[n-1]);
		}

		// establishment of the maximal likelihood ratios's  rank
		// in the array, the value of the likelihood ratio can now be
		// referenced by its rank in the array
		if (likelihood[n] > max)//!Search the max likelihood candidate
		{
			max_convolution = convolution;
			max = likelihood[n] ;
			max_rank = n ;
		}

	}//! End of the loop on the list of query pixels


	int count = 0;
  // test on the likelihood threshold if threshold==-1 then
  // the me->threshold is  selected
	double test_cvlt, test_like;
  	//! Display of the candidates, if their likelihood is higher than a threshold
  	if (range >= (numCandidates/2))
  	{
		for (int k = 0 ; k < numCandidates ; k++)
		{
			if(likelihood[ranks[k]]>threshold)
			{
// 				if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT)){
// // 					vpDisplay::displayPoint(I, i, j, vpColor::yellow);//initial point
// 					vpDisplay::display(I);
// 					vpDisplay::displayCross(I,i, j, 3,vpColor::yellow) ;
 					//vpDisplay::displayCross(I,candidateList[k].i, candidateList[k].j, 3,vpColor::red) ;
// // 					vpDisplay::displayPoint(I, candidateList[k].i, candidateList[k].j, vpColor::blue);
//
// 					test_cvlt = candidateList[k].convolution(I, me) ;
// 					test_like = likelihood[ranks[k]];
// 					vpDisplay::flush(I);
// 				}
				count++;
			}
			else
			{
				candidateList[k].normGradient = 0 ;
				if(max == 0)
					candidateList[k].suppress = 1; // contrast suppression
				else
					candidateList[k].suppress = 2; // threshold suppression
			}
// 			if (fabs(alpha) < 0.1){
// 				std::cout << "Cvlt ref : " << convlt << " , Convolution : " << candidateList[k].convlt <<" , " << last_convolution << " , Likelihood : " << likelihood[ranks[k]]<< std::endl;

// 			}
		}
// 		if (fabs(alpha) < 0.1){
// 			std::cout << count << std::endl;
// 			std::cout << std::endl;
// 		}
	}


	//  if (test_contrast)
	if(max > threshold)
	{
		//std::cout << " track mh " <<  i << " " << max_rank << " " << list_query_pixels[max_rank].i << " " << list_query_pixels[max_rank].j << std::endl;
		//if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
		//{
 			//vpDisplay::displayCross(I, list_query_pixels[max_rank].i,list_query_pixels[max_rank].j, 3, vpColor::red);
		//}

		*this = list_query_pixels[max_rank] ;//The vpMeSite is replaced by the vpMeSite of max likelihood
		normGradient =  vpMath::sqr(max_convolution);

		convlt = max_convolution;
		i_1 = ii_1; //list_query_pixels[max_rank[0]].i ;
		j_1 = jj_1; //list_query_pixels[max_rank[0]].j ;

		//vpDisplay::displayCross(I,i,j, 3, vpColor::red);
		//vpDisplay::displayCross(I,i_1,j_1, 3, vpColor::blue);

		//std::cout << " track mh " <<  i << " " << j << " " << i_1 << " " << j_1 << " " << list_query_pixels[max_rank].i << " " <<list_query_pixels[max_rank].j << " mr " <<max_rank <<std::endl;

	}
	else //none of the query sites is better than the threshold
	{
		//if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
		{
			//vpDisplay::displayPoint(I, list_query_pixels[max_rank].i,list_query_pixels[max_rank].j, vpColor::green);
		}
		normGradient = 0 ;
		if(max == 0)
			suppress = 1; // contrast suppression
		else
			suppress = 2; // threshold suppression

	}


	delete []likelihood;
	delete []ranks;
	delete []list_query_pixels ;

}

void
apMHMeSite::trackMHGrad(const vpImage<unsigned char>& I,
		const vpImage<unsigned char>& gradMap,
		const vpMe *me,
		const bool test_contraste)
{

	/*vpImageIo::writePNG(gradMap, "igrad.png");
	getchar();*/
// 	vpDisplay::displayCross(I,i, j, 3,vpColor::blue) ;

if (candidateList == NULL)
		candidateList = new apMHMeSite[numCandidates];
	//candidateVect->resize(numCandidates);

//std::cout << "numCandidates " << candidateVect->size() << std::endl;

	apMHMeSite  *list_query_pixels ;

	int * ranks = new int[numCandidates];
	for (int k = 0 ; k < numCandidates ; k++)
		ranks[k] = 0;

	int max_rank = 0;
	double  convolution = 0 ;
	double  last_convolution = 0 ;
	double  max_convolution = 0 ;
	double max = 0 ;
	double min = 0 ;
	int min_rank = 0 ;
	double contraste = 0;
  //  vpDisplay::display(I) ;
  //  vpERROR_TRACE("getclcik %d",me->range) ;
  //  vpDisplay::getClick(I) ;

  // range = +/- range of pixels within which the correspondent
  // of the current pixel will be sought
	int range  = me->range ;

	if ((range*2 + 1 < numCandidates)&&(range>0))
	{
		std::cout << "WARNING: 2* range +1 < numCandidates. Range = " << range << " is reset to numCandidates/2 = " << numCandidates/2 << std::endl;
		range = numCandidates/2;
	}
	//  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
	list_query_pixels = getQueryList(I, range) ;

	double  contraste_max = 1 + me->mu2 ;
	double  contraste_min = 1 - me->mu1 ;

	// array in which likelihood ratios will be stored
	double  *likelihood = new double[ 2 * range + 1 ] ;

	int ii_1 = i ;//!What for?
	int jj_1 = j ;
	i_1 = i ;
	j_1 = j ;
	double threshold;
	threshold = me->threshold ;

  //    std::cout <<"---------------------"<<std::endl ;
  	//! Loop on the list of query pixels. Find the best candidates in terms of likelihood value
	bool up = false;//indicate if likelihood is increasing
	for(int n = 0 ; n < 2 * range + 1 ; n++)
	{

		//   convolution results
		last_convolution = convolution;
		convolution = list_query_pixels[n].convolution(I, me) ;

		// luminance ratio of reference pixel to potential correspondent pixel
		// the luminance must be similar, hence the ratio value should
		// lay between, for instance, 0.5 and 1.5 (parameter tolerance)

		if( test_contraste )
		{
			// Include this to eliminate temporal calculation
			if (convlt==0)
			{
				std::cout << "vpMeSite::track : Division by zero  convlt = 0" << std::endl ;
				delete []list_query_pixels ;
				delete []likelihood;
				delete []ranks;
				//throw(vpTrackingException(vpTrackingException::initializationError,
					//		"Division by zero")) ;
			}

			contraste = fabs(convolution / convlt) ;
			// likelihood ratios
			if((contraste > contraste_min) && (contraste < contraste_max))
				likelihood[n] = fabs(convolution + convlt ) ;
			else
				likelihood[n] = 0 ;
		}
		else
			likelihood[n] = fabs(2*convolution) ;

		//std::cout << " track mh " << likelihood[1]  << " " << likelihood[2] << " " << likelihood[3] << std::endl;

 		//std::cout << "convlt : " << convlt << " , Convolution : " << convolution << " , Likelihood : " << likelihood[n]<< " ncand "  << std::endl;
		//To obtain the numCandidates best candidates of the list of vpMeSite: (tested)
		if (n < numCandidates)
		{
			if (n==0)
				min = likelihood[n];
			else
			{
				if (likelihood[n] < min)
				{
					min_rank = n;
					min = likelihood[n];
				}
				if (likelihood[n] > likelihood[n-1])
					up = true;
			}

			candidateList[n] = list_query_pixels[n];
			ranks[n] = n;
			candidateList[n].convlt = convolution;
			candidateList[min_rank].normGradient =  vpMath::sqr(convolution);
		}
		else
		{
			if ((likelihood[n] < likelihood[n-1])&&(up==true))//changement de direction, n-1 était un max local
			{
				if (likelihood[n-1]>min)
				{
					candidateList[min_rank] = list_query_pixels[n-1];
					ranks[min_rank] = n-1;
					candidateList[min_rank].convlt = last_convolution;
					candidateList[min_rank].normGradient =  vpMath::sqr(last_convolution);
	// 				Compute new min of candidateList
					min = likelihood[ranks[0]];
					min_rank=0;
					for (int k = 1 ; k < numCandidates ; k++)
					{
						if (likelihood[ranks[k]] < min)
						{
							min = likelihood[ranks[k]];
							min_rank = k;
						}
					}
				}
			}
			up = (likelihood[n] >= likelihood[n-1]);
		}

		// establishment of the maximal likelihood ratios's  rank
		// in the array, the value of the likelihood ratio can now be
		// referenced by its rank in the array
		if (likelihood[n] > max)//!Search the max likelihood candidate
		{
			max_convolution = convolution;
			max = likelihood[n] ;
			max_rank = n ;
		}

	}//! End of the loop on the list of query pixels


	int count = 0;
  // test on the likelihood threshold if threshold==-1 then
  // the me->threshold is  selected
	double test_cvlt, test_like;
  	//! Display of the candidates, if their likelihood is higher than a threshold

  	if (range >= (numCandidates/2))
  	{
		for (int k = 0 ; k < numCandidates ; k++)
		{
			//std::cout << " k " << k << " theta g " << candidateList[k].thetaGrad << " alpha " << candidateList[k].alpha << std::endl;
			if(likelihood[ranks[k]]>threshold)
			{
// 				if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT)){
// // 					vpDisplay::displayPoint(I, i, j, vpColor::yellow);//initial point
// 					vpDisplay::display(I);
// 					vpDisplay::displayCross(I,i, j, 3,vpColor::yellow) ;
 					//vpDisplay::displayCross(I,candidateList[k].i, candidateList[k].j, 3,vpColor::red) ;
 					//std::cout << " track mh " <<  candidateList[0].i<< std::endl;
// // 					vpDisplay::displayPoint(I, candidateList[k].i, candidateList[k].j, vpColor::blue);
//
// 					test_cvlt = candidateList[k].convolution(I, me) ;
// 					test_like = likelihood[ranks[k]];
// 					vpDisplay::flush(I);
// 				}
				candidateList[k].thetaGrad = M_PI*(((double)gradMap[candidateList[k].i][candidateList[k].j])/255-0.5);

				/*if(candidateList[k].i == 112 && candidateList[k].j == 281)
					std::cout << " ii " << candidateList[k].i << " jj " << candidateList[k].j << " i " << i << "j " << j << " gradmap " << candidateList[k].thetaGrad<< std::endl;*/
				count++;
			}
			else
			{
				//vpDisplay::displayCross(I,candidateList[k].i, candidateList[k].j, 3,vpColor::red) ;
				candidateList[k].normGradient = 0 ;
				if(max == 0)
					candidateList[k].suppress = 1; // contrast suppression
				else
					candidateList[k].suppress = 2; // threshold suppression
			}
// 			if (fabs(alpha) < 0.1){
// 				std::cout << "Cvlt ref : " << convlt << " , Convolution : " << candidateList[k].convlt <<" , " << last_convolution << " , Likelihood : " << likelihood[ranks[k]]<< std::endl;

// 			}
		}
// 		if (fabs(alpha) < 0.1){
// 			std::cout << count << std::endl;
// 			std::cout << std::endl;
// 		}
	}


	//  if (test_contrast)
	if(max > threshold)
	{
		//std::cout << " track mh " <<  i << " " << max_rank << " " << list_query_pixels[max_rank].i << " " << list_query_pixels[max_rank].j << std::endl;
		//if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
		//{
 			//vpDisplay::displayCross(I, list_query_pixels[max_rank].i,list_query_pixels[max_rank].j, 3, vpColor::red);
		//}

		*this = list_query_pixels[max_rank] ;//The vpMeSite is replaced by the vpMeSite of max likelihood
		normGradient =  vpMath::sqr(max_convolution);

		convlt = max_convolution;
		i_1 = ii_1; //list_query_pixels[max_rank[0]].i ;
		j_1 = jj_1; //list_query_pixels[max_rank[0]].j ;

		//vpDisplay::displayCross(I,i,j, 3, vpColor::red);
		//vpDisplay::displayCross(I,i_1,j_1, 3, vpColor::blue);

		//std::cout << " track mh " <<  i << " " << j << " " << i_1 << " " << j_1 << " " << list_query_pixels[max_rank].i << " " <<list_query_pixels[max_rank].j << " mr " <<max_rank <<std::endl;

	}
	else //none of the query sites is better than the threshold
	{
		//if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
		{
			//vpDisplay::displayPoint(I, list_query_pixels[max_rank].i,list_query_pixels[max_rank].j, vpColor::green);
		}
		normGradient = 0 ;
		if(max == 0)
			suppress = 1; // contrast suppression
		else
			suppress = 2; // threshold suppression

	}


	delete []likelihood;
	delete []ranks;
	delete []list_query_pixels ;

}

int apMHMeSite::operator!=(const apMHMeSite &m)
{
  return((m.i != i) || (m.j != j)) ;
}


std::ostream& operator<<(std::ostream& os, apMHMeSite& vpMeS)
    {
      return (os << "Alpha: " << vpMeS.alpha
	      << "  Convolution: " << vpMeS.convlt
	      << "  Flag: " << vpMeS.suppress
	      << "  Weight: " << vpMeS.weight );
    }


