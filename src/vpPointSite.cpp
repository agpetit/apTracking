/****************************************************************************
 *
 * $Id: vpPointSite.cpp 2807 2010-09-14 10:14:54Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 * Andrew Comport
 *
 *****************************************************************************/

/*!
  \file vpPointSite.cpp
  \brief Moving edges
*/

#include <stdlib.h>

#include "vpPointSite.h"
#include <visp/vpMe.h>
#include <visp/vpTrackingException.h>
#include <visp/vpImageIo.h>

#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 0


#ifndef DOXYGEN_SHOULD_SKIP_THIS
static
bool horsImage( int i , int j , int half , int rows , int cols)
{
  return((i < half + 1) || ( i > (rows - half - 3) )||(j < half + 1) || (j > (cols - half - 3) )) ;
}
#endif

void
vpPointSite::init()
{
  // Site components
  alpha =  0.0 ;
  convlt = 0.0 ;
  suppress = 0;
  weight=-1;

	candidateList = NULL;
	numCandidates = 3;
	//candidateVect.resize(1);

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
}

vpPointSite::vpPointSite(double ip, double jp)
{
  init() ;

  selectDisplay = NONE ;
  i = vpMath::round(ip) ;
  j = vpMath::round(jp) ;
  ifloat = ip ;
  jfloat = jp ;
}

// More an Update than init
// For points in meter form (to avoid approximations)
void
vpPointSite::init(double ip, double jp, double alphap)
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
vpPointSite::init(double ip, double jp, double alphap, double convltp)
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
	//candidateVect.resize(1);

  stdprofile[0]=0;
  stdprofile[1]=1.33;
  stdprofile[2]=12.2;
  stdprofile[3]=36.5;
  stdprofile[4]=12.2;
  stdprofile[5]=1.33;
  stdprofile[6]=0;
  profile[0]=0;
  profile[1]=1.33;
  profile[2]=12.2;
  profile[3]=36.5;
  profile[4]=12.2;
  profile[5]=1.33;
  profile[6]=0;
}
// initialise with convolution and sign
void
vpPointSite::init(double ip, double jp, double alphap, double convltp, int sign)
{
  selectDisplay = NONE ;
  ifloat = ip ;
  i= (int)ip ;
  jfloat = jp ;
  j =(int)jp  ;
  alpha = alphap ;
  convlt = convltp;
  mask_sign = sign ;
	candidateList = NULL;
	numCandidates = 3;
//candidateVect.resize(1);

  v = 0 ;
  i_1 = 0 ;
  j_1 = 0 ;

  stdprofile[0]=0;
  stdprofile[1]=1.33;
  stdprofile[2]=12.2;
  stdprofile[3]=36.5;
  stdprofile[4]=12.2;
  stdprofile[5]=1.33;
  stdprofile[6]=0;
  profile[0]=0;
  profile[1]=1.33;
  profile[2]=12.2;
  profile[3]=36.5;
  profile[4]=12.2;
  profile[5]=1.33;
  profile[6]=0;
}

// ===================================================================
/*!
 * Construct and return the list of vpPointSite along the normal to the contour, in the given range.
 * \pre : ifloat, jfloat, and the direction of the normal (alpha) have to be set. 
 * \param I : Image in which the display is performed.
 * \param range :  +/- the range within which the pixel's correspondent will be sought
 * \return Pointer to the list of query sites
 */
// ===================================================================

vpPointSite*
vpPointSite::getQueryList(const vpImage<unsigned char> &I, const int range)
{

  int   k ;

  int n;
  double ii , jj ;
  vpPointSite *list_query_pixels ;
  list_query_pixels =  NULL ;

  // Size of query list includes the point on the line
  list_query_pixels = new vpPointSite[2 * range + 1] ;

  // range : +/- the range within which the pixel's
  //correspondent will be sought

  double salpha = sin(alpha);
  double calpha = cos(alpha);
  //std::cout<<"alpha "<<(180/M_PI)*alpha<<std::endl;
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
    vpPointSite pel ;
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
vpPointSite::getSign(const vpImage<unsigned char> &I, const int range)
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
vpPointSite::convolution(const vpImage<unsigned char>&I, const  vpMe *me)
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

    const int ihalf = i - half;
    const int jhalf = j - half;
    const vpMatrix &mask = me->mask[index_mask];
    const int mask_size = me->mask_size;
    for(int a = 0 ; a < mask_size ; ++a)
    {
        const int ihalfa = ihalf + a ;
        const double * const mask_ptr = mask[a];
        const unsigned char * const I_ptr = I[ihalfa] + jhalf;
        for(int b = 0 ; b < mask_size ; ++b)
        {
            conv += mask_ptr[b] * I_ptr[b];
                    //	  I(i-half+a,j-half+b) ;
//                    I(ihalfa,jhalf+b);
        }
    }
    conv *= mask_sign;

  }

  return(conv) ;
}


/*!

  Specific function for ME.

  \warning To display the moving edges graphics a call to vpDisplay::flush()
  is needed.

*/
void
vpPointSite::track(const vpImage<unsigned char>& I,
		const vpMe *me,
		const bool test_contraste)
{
//   vpPointSite  *list_query_pixels ;
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
// 	      std::cout << "vpPointSite::track : Division by zero  convlt = 0" << std::endl ;
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
//       *this = list_query_pixels[max_rank] ;//The vpPointSite is replaced by the vpPointSite of max likelihood
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

  vpPointSite  *list_query_pixels ;
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

  int range  = me->range;
 //int range1 = range + 6 ;
  //int range1 = range;


  //  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
  list_query_pixels = getQueryList(I, range) ;

  //list_query_pixels = getQueryList(I, range1) ;

  //double *deriv_query_pixels= new double[2*range1 -5];

    /*for(int m = 0 ; m < 2 * range1 - 5 ;m++)
    {
  	  double deriv=(2047.0 *(I[list_query_pixels[m+2].i][list_query_pixels[m+2].j] - I[list_query_pixels[m+4].i][list_query_pixels[m+4].j])
               +913.0 *(I[list_query_pixels[m+1].i][list_query_pixels[m+1].j] - I[list_query_pixels[m+5].i][list_query_pixels[m+5].j])
               +112.0 *(I[list_query_pixels[m].i][list_query_pixels[m].j] - I[list_query_pixels[m+6].i][list_query_pixels[m+6].j]))/8418.0;
  	  deriv_query_pixels[m]=sqrt(deriv*deriv);
  	  //std::cout<<deriv_query_pixels[m]<<std::endl;
    }*/


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

	  //convolution = list_query_pixels[n+6].convolution(I, me) ;

      // luminance ratio of reference pixel to potential correspondent pixel
      // the luminance must be similar, hence the ratio value should
      // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
      if( test_contraste )
      {
	likelihood[n] = fabs(convolution + convlt);
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
      //if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
	{

		ip.set_i( list_query_pixels[max_rank].i );
			  ip.set_j( list_query_pixels[max_rank].j );

		/*ip.set_i( list_query_pixels[max_rank+6].i );
		ip.set_j( list_query_pixels[max_rank+6].j );*/
	  //vpDisplay::displayPoint(I, ip, vpColor::blue);
	}
		
      *this = list_query_pixels[max_rank] ;//The vpPointSite2 is replaced by the vpPointSite2 of max likelihood
	  //*this = list_query_pixels[max_rank+6] ;
      normGradient =  vpMath::sqr(max_convolution);

      /*profile[0]=deriv_query_pixels[max_rank];
      profile[1]=deriv_query_pixels[max_rank+1];
      profile[2]=deriv_query_pixels[max_rank+2];
      profile[3]=deriv_query_pixels[max_rank+3];
      profile[4]=deriv_query_pixels[max_rank+4];
      profile[5]=deriv_query_pixels[max_rank+5];
      profile[6]=deriv_query_pixels[max_rank+6];*/

      convlt = max_convolution;
      i_1 = ii_1; //list_query_pixels[max_rank].i ;
      j_1 = jj_1; //list_query_pixels[max_rank].j ;
      delete []list_query_pixels ;
      delete []likelihood;
      //delete []deriv_query_pixels;
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
	{suppress = 1; // contrast suppression
      //std::cout << " contrast " << std::endl;
	}
      else
	{suppress = 2; // threshold suppression
      //std::cout << " threshold " << std::endl;
	}
	
      delete []list_query_pixels ;
      delete []likelihood; // modif portage
      //delete []deriv_query_pixels;
    }
}

void
vpPointSite::trackMH(const vpImage<unsigned char>& I,
		const vpMe *me,
		const bool test_contraste)
{
	if (candidateList == NULL)
			{
		candidateList = new vpPointSite[numCandidates];}

		vpPointSite  *list_query_pixels ;

		int * ranks = new int[numCandidates];
		for (int k = 0 ; k < numCandidates ; k++)
			ranks[k] = 0;

		int max_rank = -1;
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
					throw(vpTrackingException(vpTrackingException::initializationError,
								"Division by zero")) ;
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


	// 		std::cout << "convlt : " << convlt << " , Convolution : " << convolution << " , Likelihood : " << likelihood[n]<< std::endl;
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
			if (likelihood[n] > max && likelihood[n]>threshold)//!Search the max likelihood candidate
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
vpImagePoint ip;
		//  if (test_contrast)
		if(max_rank >= 0)
		{
			//if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
			{
				ip.set_i( list_query_pixels[max_rank].i );
				ip.set_j( list_query_pixels[max_rank].j );
                           //vpDisplay::displayPoint(I, ip, vpColor::blue);
                            vpDisplay::displayCross(I, ip, 3, vpColor::blue);
			}

			*this = list_query_pixels[max_rank] ;//The vpMeSite is replaced by the vpMeSite of max likelihood
			normGradient =  vpMath::sqr(max_convolution);

			convlt = max_convolution;
			i_1 = ii_1; //list_query_pixels[max_rank[0]].i ;
			j_1 = jj_1; //list_query_pixels[max_rank[0]].j ;
			/*delete []likelihood;
			delete []ranks;
			delete []list_query_pixels ;*/

		}
		else //none of the query sites is better than the threshold
		{
                        if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
			{
				vpDisplay::displayPoint(I, list_query_pixels[max_rank].i,list_query_pixels[max_rank].j, vpColor::green);
			}
			normGradient = 0 ;
			if(max_rank == -1)
				suppress = 1; // contrast suppression
			else
				suppress = 2; // threshold suppression

		}

		delete []likelihood;
		delete []ranks;
		delete []list_query_pixels ;
}



void
vpPointSite::getProfile(const vpImage<unsigned char>& I,const int size)
{
	  vpPointSite  *list_query_pixels ;
	  int range=size-1+3;
	  list_query_pixels = getQueryList(I,range);
	  double *deriv_query_pixels= new double[range+1+6];
	  double max_grad=0;
	  int max_rank=0;

	  for(int m = 0 ; m < size +6 ;m++)
	  {
		  double deriv=(2047.0 *(I[list_query_pixels[m+2].i][list_query_pixels[m+2].j] - I[list_query_pixels[m+4].i][list_query_pixels[m+4].j])
	             +913.0 *(I[list_query_pixels[m+1].i][list_query_pixels[m+1].j] - I[list_query_pixels[m+5].i][list_query_pixels[m+5].j])
	             +112.0 *(I[list_query_pixels[m].i][list_query_pixels[m].j] - I[list_query_pixels[m+6].i][list_query_pixels[m+6].j]))/8418.0;

		  deriv_query_pixels[m]=sqrt(deriv*deriv);
		  if (deriv>max_grad && m>2 && m<size+6-3)
		  {
			  max_grad=deriv;
			  max_rank=m;
		  }
		  //std::cout<<deriv_query_pixels[m]<<std::endl;
	  }

	  profile[0]=deriv_query_pixels[max_rank-3];
	  profile[1]=deriv_query_pixels[max_rank-2];
	  profile[2]=deriv_query_pixels[max_rank-1];
	  profile[3]=deriv_query_pixels[max_rank];
	  profile[4]=deriv_query_pixels[max_rank+1];
	  profile[5]=deriv_query_pixels[max_rank+2];
	  profile[6]=deriv_query_pixels[max_rank+3];
	  //std::cout<< " ok "<<profile[0]<<" "<<profile[1]<<" "<<profile[2]<<" "<<profile[3]<<" "<<profile[4]<<" "<<profile[5]<<" "<<profile[6]<<std::endl;




}

void
vpPointSite::getProfile1(const vpImage<unsigned char>& I,const int size)
{
	  vpPointSite  *list_query_pixels0 ;
	  list_query_pixels0 = getQueryList(I,size);
	  double *deriv_query_pixels0= new double[2*size -5];
	  profile1 = new double[2* size +1];
	  profile2 = new double[2* size +1];
	  double max_grad=0;
	  int max_rank=0;
	  //std::cout<< " ok " << size << std::endl;

	  for(int m = 0 ; m <  2 * size + 1 ;m++)
	  {
		  /*double deriv=(2047.0 *(I[list_query_pixels0[m+2].i][list_query_pixels0[m+2].j] - I[list_query_pixels0[m+4].i][list_query_pixels0[m+4].j])
	             +913.0 *(I[list_query_pixels0[m+1].i][list_query_pixels0[m+1].j] - I[list_query_pixels0[m+5].i][list_query_pixels0[m+5].j])
	             +112.0 *(I[list_query_pixels0[m].i][list_query_pixels0[m].j] - I[list_query_pixels0[m+6].i][list_query_pixels0[m+6].j]))/8418.0;

		  deriv_query_pixels0[m]=deriv;*/
		  //std::cout<< " ok1 " << m << std::endl;
		  profile1[m] = I[list_query_pixels0[m].i][list_query_pixels0[m].j];
		  //profile1[m] = 100 + m;
		  //profile2[m] = 95 + m;
		  //std::cout<<deriv_query_pixels0[m]<<std::endl;
	  }

	  /*for(int m = 0 ; m <  2 * size + 1  ;m++)
	  	  {

	  		  //std::cout<< " ok1 " << m << std::endl;
	  		  //profile1[m] = I[list_query_pixels0[m].i][list_query_pixels0[m].j];
	  		  profile1[m] = 100 + m;
	  		  profile2[m] = 95 + m;
	  		  //std::cout<<deriv_query_pixels0[m]<<std::endl;
	  	  }*/


	  //std::cout<< " ok "<< profile1[0]<<" "<< profile1[1] <<" "<<profile1[2]<<" "<<profile1[3]<<" "<<profile1[4]<<" "<<profile1[5]<<" "<<profile1[6]<< " " << profile1[7]<<" "<< profile1[8] <<" "<<profile1[9]<<" "<<profile1[10]<<" "<<profile1[11]<<" "<<profile1[12]<<" "<<profile1[13]<< std::endl;

	  delete []deriv_query_pixels0;
	  delete []list_query_pixels0;


}




void
vpPointSite::trackProfile(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec, const vpMe *me)
{

   	vpPointSite  *list_query_pixels;
	  int  max_rank =-1;
	  int max_rank1=-1;
	  int max_rank2 =-1;
	  double  convolution = 0;
	  double  max_convolution = 0;
	  double max = 0 ;
	  double correl_ = 0;
	  double correl;
	  double stdcorrel;

	  //  vpDisplay::display(I) ;
	  //  vpERROR_TRACE("getclcik %d",me->range) ;
	  //  vpDisplay::getClick(I) ;

	  // range = +/- range of pixels within which the correspondent
	  // of the current pixel will be sought
	  int range  = 2*(me->range)+3;

	  //std::cout << " range " << 2*(me->range)+3 << std::endl;

	  //  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
	  list_query_pixels = getQueryList(I, range);

	  //std::cout<< " ok00 " << list_query_pixels[0].i << " " << list_query_pixels[0].j << std::endl;

	  getProfile1(Iprec, range);

	  double *deriv_query_pixels= new double[2*range -5];

	  for(int m = 0 ; m < 2 * range - 5 ;m++)
	  {
		  double deriv=(2047.0 *(I[list_query_pixels[m+2].i][list_query_pixels[m+2].j] - I[list_query_pixels[m+4].i][list_query_pixels[m+4].j])
	             +913.0 *(I[list_query_pixels[m+1].i][list_query_pixels[m+1].j] - I[list_query_pixels[m+5].i][list_query_pixels[m+5].j])
	             +112.0 *(I[list_query_pixels[m].i][list_query_pixels[m].j] - I[list_query_pixels[m+6].i][list_query_pixels[m+6].j]))/8418.0;
		  deriv_query_pixels[m]=deriv;
		  //std::cout<<deriv_query_pixels[m]<<std::endl;
	  }

	  /*for(int m = 0 ; m < 2 * range - 5 ;m++)
	  {
		  double deriv=(2047.0 *(profile2[m+2] -profile2[m+4])
	             +913.0 *(profile2[m+1] - profile2[m+5])
	             +112.0 *(profile2[m] - profile2[m+6]))/8418.0;
		  deriv_query_pixels[m]=deriv;
		  //std::cout<<deriv_query_pixels[m]<<std::endl;
	  }*/

	  //std::cout << " range " << profile1[0]<< std::endl;

	  // array in which likelihood ratios will be stored
	  double  *correlation= new double[ 2*range-5 ] ;
	  double  *stdcorrelation= new double[ 2*range-5 ] ;

	  int ii_1 = i;
	  int jj_1 = j;
	  i_1 = i ;
	  j_1 = j ;
	  double threshold;
	  double delta = 0;
	  int niter = 10;
	  double hessian;
	  int rank = 0;
	  double error;

	  for (int k = 0; k < niter ; k++)
	  {
		  hessian = 0;
		  delta = 0;
		  error = 0;
		  for(int m =  me->range ; m < 2*(me->range)+1 + me->range;m++)
		  {
			  //std::cout << " m rank " << m+(int)rank << std::endl;
			  hessian += deriv_query_pixels[m+(int)rank]*deriv_query_pixels[m+(int)rank];
		  }
		  for(int m =  me->range ; m < 2*(me->range)+1 + me->range;m++)
		  {
			  //std::cout << " deriv " << deriv_query_pixels[m+(int)rank] <<" profile " << profile1[m] << " " << (double)I[list_query_pixels[m+(int)rank].i][list_query_pixels[m+(int)rank].j] << std::endl;
			  //std::cout << " rank " << m+(int)rank << " deriv " << (double)list_query_pixels[m+(int)rank].i << " " << (double)list_query_pixels[m+(int)rank].j << std::endl;
			  delta += deriv_query_pixels[m+(int)rank]*(profile1[m] - I[list_query_pixels[m+(int)rank].i][list_query_pixels[m+(int)rank].j]);
			  error += (-profile1[m] + I[list_query_pixels[m+(int)rank].i][list_query_pixels[m+(int)rank].j]);
			  //std::cout << " diff " <<profile1[m] << " im " << (double)I[list_query_pixels[m+(int)rank].i][list_query_pixels[m+(int)rank].j] << " deriv " << deriv_query_pixels[m+(int)rank] << " delta " << delta << std::endl;
		  }
		  //std::cout << " delta " << delta << std::endl;
		  delta /= hessian;
		  if (abs(delta) > 10)
			  delta = 0;
		  //std::cout <<" k " << k <<" delta " << delta << " hessian " << hessian << std::endl;
		  rank -= (int)delta;
	  }


	 /* for (int k = 0; k < niter ; k++)
	  {
		  hessian = 0;
		  delta = 0;
		  error = 0;
		  for(int m =  me->range ; m < 2*(me->range)+1 + me->range;m++)
		  {
			  hessian += deriv_query_pixels[m+(int)rank]*deriv_query_pixels[m+(int)rank];
			  std::cout << "m rank " << m+(int)rank <<" deriv " << deriv_query_pixels[m+(int)rank]*deriv_query_pixels[m+(int)rank] << std::endl;
		  }
		  for(int m =  me->range ; m < 2*(me->range)+1 + me->range;m++)
		  {
			  std::cout << " profile " << profile1[m] << " " << (double)profile2[m+(int)rank] << std::endl;
			  //std::cout << " rank " << m+(int)rank << " deriv " << (double)list_query_pixels[m+(int)rank].i << " " << (double)list_query_pixels[m+(int)rank].j << std::endl;
			  delta += deriv_query_pixels[m+(int)rank]*(profile1[m] - profile2[m+(int)rank]);
			  error += (-profile1[m] + profile2[m+(int)rank]);
			  //std::cout << " diff " <<profile1[m] << " im " << (double)I[list_query_pixels[m+(int)rank].i][list_query_pixels[m+(int)rank].j] << " deriv " << deriv_query_pixels[m+(int)rank] << " delta " << delta << std::endl;
		  }
		  //std::cout << " delta " << delta << std::endl;
		  delta /= hessian;
		  std::cout <<" k " << k <<" delta " << delta << " hessian " << hessian << std::endl;
		  rank -= (int)delta;
	  }*/

	  max_rank = 2*(me->range)+ 3 + rank;
	  //std::cout << " delta " << rank << " max_rank " << max_rank << std::endl;


	  vpImagePoint ip;

	  //  if (test_contrast)
	  if(max_rank >= 0)
	  {
		  //ip.set_i( list_query_pixels[max_rank].i );
		  //ip.set_j( list_query_pixels[max_rank].j );
		  //vpDisplay::displayPoint(I, ip, vpColor::red);

	      *this = list_query_pixels[max_rank];//The vpMeSite2 is replaced by the vpMeSite2 of max likelihood
	      //normGradient =  vpMath::sqr(max_convolution);

	      convlt = error;
	      /*profile[0]=deriv_query_pixels[max_rank-6];
	      profile[1]=deriv_query_pixels[max_rank-5];
	      profile[2]=deriv_query_pixels[max_rank-4];
	      profile[3]=deriv_query_pixels[max_rank-3];
	      profile[4]=deriv_query_pixels[max_rank-2];
	      profile[5]=deriv_query_pixels[max_rank-1];
	      profile[6]=deriv_query_pixels[max_rank];*/
	      i_1 = ii_1; //list_query_pixels[max_rank].i ;
	      j_1 = jj_1; //list_query_pixels[max_rank].j ;
	      delete []list_query_pixels;
	      delete []deriv_query_pixels;
	      suppress=0;
	  }
	  else
	  {
		  //std::cout<<" ok "<<std::endl;
	      suppress=2;
	      delete []list_query_pixels;
	      delete []deriv_query_pixels;
	  }


}

void
vpPointSite::track2(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec,
		const vpMe *me,
		const bool test_correl)
{

  vpPointSite  *list_query_pixels ;
  int  max_rank =-1;
  int max_rank1=-1;
  int max_rank2 =-1;
  double  convolution = 0;
  double  max_convolution = 0;
  double max = 0 ;
  double correl_ = 0;
  double correl;
  double stdcorrel;

  //  vpDisplay::display(I) ;
  //  vpERROR_TRACE("getclcik %d",me->range) ;
  //  vpDisplay::getClick(I) ;

  // range = +/- range of pixels within which the correspondent
  // of the current pixel will be sought
  int range  = (me->range)+6;

  //  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
  list_query_pixels = getQueryList(I, range);

  double *deriv_query_pixels= new double[2*range -5];

  for(int m = 0 ; m < 2 * range - 5 ;m++)
  {
	  double deriv=(2047.0 *(I[list_query_pixels[m+2].i][list_query_pixels[m+2].j] - I[list_query_pixels[m+4].i][list_query_pixels[m+4].j])
             +913.0 *(I[list_query_pixels[m+1].i][list_query_pixels[m+1].j] - I[list_query_pixels[m+5].i][list_query_pixels[m+5].j])
             +112.0 *(I[list_query_pixels[m].i][list_query_pixels[m].j] - I[list_query_pixels[m+6].i][list_query_pixels[m+6].j]))/8418.0;
	  deriv_query_pixels[m]=sqrt(deriv*deriv);
	  //std::cout<<deriv_query_pixels[m]<<std::endl;
  }


  // array in which likelihood ratios will be stored
  double  *correlation= new double[ 2*range-5 ] ;
  double  *stdcorrelation= new double[ 2*range-5 ] ;

  int ii_1 = i ;
  int jj_1 = j ;
  i_1 = i ;
  j_1 = j ;
  double threshold;

  //threshold = me->threshold ;
  //double diff = 1e6;

  //    std::cout <<"---------------------"<<std::endl ;

  //getProfile(Iprec,7);


  //std::cout<< " ok1  "<<profile[0]<<" "<<profile[1]<<" "<<profile[2]<<" "<<profile[3]<<" "<<profile[4]<<" "<<profile[5]<<" "<<profile[6]<<std::endl;

	//std::cout<< " ok1  "<<stdprofile[0]<<" "<<stdprofile[1]<<" "<<stdprofile[2]<<" "<<stdprofile[3]<<" "<<stdprofile[4]<<" "<<stdprofile[5]<<" "<<stdprofile[6]<<std::endl;
  for(int n = 3 ; n < 2 * range-8 ; n++)
  {
      //   convolution results
	  double norm = deriv_query_pixels[n-3]*deriv_query_pixels[n-3] + deriv_query_pixels[n-2]*deriv_query_pixels[n-2] + deriv_query_pixels[n-1]*deriv_query_pixels[n-1] + deriv_query_pixels[n]*deriv_query_pixels[n] + deriv_query_pixels[n+1]*deriv_query_pixels[n+1]+
    		  deriv_query_pixels[n+2]*deriv_query_pixels[n+2] + deriv_query_pixels[n+3]*deriv_query_pixels[n+3];
	  double normprof = profile[0]*profile[0] + profile[1]*profile[1] + profile[2]*profile[2] + profile[3]*profile[3] + profile[4]*profile[4]+
			  profile[5]*profile[5] + profile[6]*profile[6];
	  double normstdprof = stdprofile[0]*stdprofile[0] + stdprofile[1]*stdprofile[1] + stdprofile[2]*stdprofile[2] + stdprofile[3]*stdprofile[3] + stdprofile[4]*stdprofile[4]+
			  stdprofile[5]*stdprofile[5] + stdprofile[6]*stdprofile[6];
      correl = (deriv_query_pixels[n-3]*profile[0] + deriv_query_pixels[n-2]*profile[1] + deriv_query_pixels[n-1]*profile[2] + deriv_query_pixels[n]*profile[3] + deriv_query_pixels[n+1]*profile[4]+
    		  deriv_query_pixels[n+2]*profile[5] + deriv_query_pixels[n+3]*profile[6])/(sqrt(norm)*sqrt(normprof));
      stdcorrel = (deriv_query_pixels[n-3]*stdprofile[0]+deriv_query_pixels[n-2]*stdprofile[1]+deriv_query_pixels[n-1]*stdprofile[2]+deriv_query_pixels[n]*stdprofile[3]+deriv_query_pixels[n+1]*stdprofile[4]+
    		   deriv_query_pixels[n+2]*stdprofile[5]+deriv_query_pixels[n+3]*stdprofile[6])/(sqrt(norm)*sqrt(normstdprof));


      // luminance ratio of reference pixel to potential correspondent pixel
      // the luminance must be similar, hence the ratio value should
      // lay between, for instance, 0.5 and 1.5 (parameter tolerance)

	correlation[n] =correl;
	stdcorrelation[n]=stdcorrel;
	//std::cout<<"correl "<<correl << "  std correl "<< stdcorrel<< " "<<max<<std::endl;
	//std::cout<<"profile[0] "<<profile[3] << " profile[1] "<< " "<<std::endl;
	//std::cout<<"deriv "<<deriv_query_pixels[n-3]<< "  "<<deriv_query_pixels[n-2]<< "  "<<deriv_query_pixels[n-1]<<"  "<<deriv_query_pixels[n]<<"  "<<deriv_query_pixels[n+1]<<"  "<<deriv_query_pixels[n+2]<<"  "<<deriv_query_pixels[n+3]<<std::endl;


	if(test_correl)
	{

	/*if (stdcorrelation[n]> max && stdcorrelation[n]>0.6)
	{
	    //diff = fabs(1-contraste);
	    max_convolution= stdcorrel;
	    max = stdcorrelation[n];

	    max_rank = n+3;
	    max_rank2 = max_rank1;
	    max_rank1 = max_rank;
	  }*/
		if (deriv_query_pixels[n]>3)
		{
			if (stdcorrelation[n]> max && stdcorrelation[n]>0.6)
			{
				//std::cout << " ok " << std :: endl;
			    //diff = fabs(1-contraste);
			    max_convolution= stdcorrel;
			    max = stdcorrelation[n];

			    max_rank = n+3;
			    max_rank2 = max_rank1;
			    max_rank1 = max_rank;
			  }
	/*if (correlation[n]> max && correlation[n]>0.4)
	{
	    //diff = fabs(1-contraste);
	    max_convolution= correl;
	    max = correlation[n];

	    max_rank = n+3;
	    max_rank2 = max_rank1;
	    max_rank1 = max_rank;
	  }*/
	}
	}
	else
	{
		if (stdcorrelation[n]> max)
		{
		    //diff = fabs(1-contraste);
		    max_convolution= stdcorrel;
		    max = stdcorrelation[n];

		    max_rank = n+3;
		    max_rank2 = max_rank1;
		    max_rank1 = max_rank;
		  }

		if (correlation[n]> max)
		{
		    //diff = fabs(1-contraste);
		    max_convolution= correl;
		    max = correlation[n];

		    max_rank = n+3;
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
	  //ip.set_i( list_query_pixels[max_rank].i );
	  //ip.set_j( list_query_pixels[max_rank].j );
	  //vpDisplay::displayPoint(I, ip, vpColor::red);

      *this = list_query_pixels[max_rank];//The vpMeSite2 is replaced by the vpMeSite2 of max likelihood
      //normGradient =  vpMath::sqr(max_convolution);

      convlt = max_convolution;
      /*profile[0]=deriv_query_pixels[max_rank-6];
      profile[1]=deriv_query_pixels[max_rank-5];
      profile[2]=deriv_query_pixels[max_rank-4];
      profile[3]=deriv_query_pixels[max_rank-3];
      profile[4]=deriv_query_pixels[max_rank-2];
      profile[5]=deriv_query_pixels[max_rank-1];
      profile[6]=deriv_query_pixels[max_rank];*/
      i_1 = ii_1; //list_query_pixels[max_rank].i ;
      j_1 = jj_1; //list_query_pixels[max_rank].j ;
      delete []list_query_pixels;
      delete []deriv_query_pixels;
      delete []correlation;
      delete []stdcorrelation;
      suppress=0;
  }
  else
  {
	  //std::cout<<" ok "<<std::endl;
      suppress=2;
      delete []list_query_pixels;
      delete []deriv_query_pixels;
      delete []correlation;
      delete []stdcorrelation;
  }

}


int vpPointSite::operator!=(const vpPointSite &m)
{
  return((m.i != i) || (m.j != j)) ;

}

std::ostream& operator<<(std::ostream& os, vpPointSite& vpMeS)
    {
      return (os << "Alpha: " << vpMeS.alpha
	      << "  Convolution: " << vpMeS.convlt 
	      << "  Flag: " << vpMeS.suppress
	      << "  Weight: " << vpMeS.weight );
    }
