/****************************************************************************
 *
 * $Id: vpPointSite.h 2807 2010-09-14 10:14:54Z fspindle $
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
	\file vpPointSite.h
	\brief Moving edges
*/



#ifndef vpPointSite_H
#define vpPointSite_H

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpMe.h>
#include <vector>

/*!
  \class vpPointSite
  \ingroup TrackingImageME

  \brief Performs search in a given direction(normal) for a given
   distance(pixels) for a given 'site'. Gives the most likely site
   given the probablility from an ME mask

  - Bug fix: rewrote application of masks to use the temporal
    information instead of applying both temporal masks to the same
    image. ie: spatial -> spatio/temporal
  
  - Added new tracking function to choose the most similar edge
    amongst all edges found.

  - sample step.
 */
class VISP_EXPORT vpPointSite
{
public:
  typedef enum
    {
      NONE,
      RANGE,
      RESULT,
      RANGE_RESULT
    } vpPointSiteDisplayType;

public:
  int i,j ;
  int i_1, j_1 ;
  double ifloat, jfloat ;
  unsigned char v ;
  int mask_sign ;

  // Angle of tangent at site
  double alpha;

  // Convolution of Site in previous image
  double convlt ;
 // Convolution of Site in previous image
  double normGradient ;

  //! Flag to indicate whether point is rejected or not
  //! 1 = contrast, 2 = threshold, 3 = M-estimator, 0 = nosupp
  int suppress;

  // Uncertainty of point given as a probability between 0 and 1
  double weight;

  double profile[7];

  double *profile1;
  double *profile2;
  double stdprofile[7];

  int rangeprofile;


  //Profile of the previous image;
  //vpColVector profile;

public:
  void init() ;
  void init(double ip, double jp, double alphap) ;
  void init(double ip, double jp, double alphap, double convltp) ;
  void init(double ip, double jp, double alphap, double convltp, int sign) ;

  vpPointSite ()    {   init(); }
  vpPointSite(double ip, double jp) ;
  /*!
    Copy constructor.
  */
  vpPointSite (const vpPointSite &mesite) : candidateList(NULL)
  {
    *this = mesite;
  }
  //virtual ~vpPointSite() {} ;
  ~vpPointSite()
  {
      if (candidateList != NULL)
          delete [] candidateList;
  }

  vpPointSite &operator=(const vpPointSite &m)
  {
    i = m.i;
    j = m.j;
    i_1 = m.i_1;
    j_1 = m.j_1;
    ifloat = m.ifloat;
    jfloat = m.jfloat;
    v = m.v;
    mask_sign = m.mask_sign;
    alpha = m.alpha;
    convlt = m.convlt;
    normGradient = m.normGradient;
    suppress = m.suppress;
    weight = m.weight;
    selectDisplay = m.selectDisplay;
    numCandidates = m.numCandidates;
      //candidateVect=m.candidateVect;

    stdprofile[0]=m.stdprofile[0];
    stdprofile[1]=m.stdprofile[1];
    stdprofile[2]=m.stdprofile[2];
    stdprofile[3]=m.stdprofile[3];
    stdprofile[4]=m.stdprofile[4];
    stdprofile[5]=m.stdprofile[5];
    stdprofile[6]=m.stdprofile[6];
    profile[0]=m.profile[0];
    profile[1]=m.profile[1];
    profile[2]=m.profile[2];
    profile[3]=m.profile[3];
    profile[4]=m.profile[4];
    profile[5]=m.profile[5];
    profile[6]=m.profile[6];


    return *this ;
  }

  int operator!=(const vpPointSite  &m) ;
  
  friend std::ostream& operator<<(std::ostream& os, vpPointSite& vpMeS);

  void getSign(const vpImage<unsigned char> &I, const int range);

  double convolution(const vpImage<unsigned char>& ima, const vpMe *me);

  vpPointSite *getQueryList(const vpImage<unsigned char> &I, const int range);
  void getProfile(const vpImage<unsigned char>& I,const int size);
  void track(const vpImage<unsigned char>& I,
	     const vpMe *me,
	     const  bool test_contraste=true);

  void track2(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec,
	     const vpMe *me,
	     const  bool test_contraste=true);
  void trackMH(const vpImage<unsigned char>& I,
  		const vpMe *me,
  		const bool test_contraste);

  void getProfile1(const vpImage<unsigned char>& I,const int size);
  void trackProfile(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec, const vpMe *me );
  //vpPointSite getCandidate(int n){return candidateList[n];}

  void setNumCandidates(int Num) {numCandidates = Num;}
    int getNumCandidates() const {return numCandidates;}

  void setDisplay(vpPointSiteDisplayType select) { selectDisplay = select ; }
  
  /*!

    Compute the distance \f$ |S1 - S2| = \sqrt{(i_1-i_2)^2+(j_1-j_2)^2} \f$

    \param S1 : First site
    \param S2 : Second site

    \return the distance between the two sites.
  */
  static double distance (const vpPointSite S1, const vpPointSite S2) {
    return(sqrt(vpMath::sqr(S1.ifloat-S2.ifloat)+vpMath::sqr(S1.jfloat-S2.jfloat)));}
    
  /*!

    Compute the distance \f$ |S1 - S2| = (i_1-i_2)^2+(j_1-j_2)^2 \f$

    \param S1 : First site
    \param S2 : Second site

    \return the distance between the two sites.
  */
  static double sqrDistance (const vpPointSite S1, const vpPointSite S2) {
    return(vpMath::sqr(S1.ifloat-S2.ifloat)+vpMath::sqr(S1.jfloat-S2.jfloat));}

private:
  int numCandidates;

public :

  vpPointSite * candidateList;
  //std::vector<int> candidateVect;

private:
  vpPointSiteDisplayType selectDisplay ;
} ;

#endif
