/****************************************************************************
 *
 * $Id: vpFeaturePoint.h 2455 2010-01-07 10:24:57Z nmelchio $
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
 *   Luninance based feature .
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef apFeatureGradientOrientation_h
#define apFeatureGradientOrientation_h

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpImage.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpList.h>


/*!
  \file vpFeatureLuminance.h
  \brief Class that defines the image luminance visual feature

  for more details see
  C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/


/*!
  \class vpLuminance
  \brief Class that defines the luminance and gradient of a point

  \sa vpFeatureLuminance
*/


class apOrientation
{
 public:
  double x, y;   // point coordinates (in meter)
  double Z; // pixel depth
  double rho;
  double theta;
  double A,B,C,D;
  int i,j;


};


/*!
  \class vpFeatureLuminance
  \brief Class that defines the image luminance visual feature

  For more details see
  C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/

class apFeatureGradientOrientation : public vpBasicFeature
{
 protected:
  //! FeaturePoint depth (required to compute the interaction matrix)
  //! default Z = 1m
  double Z ;

  int nbr ;
  int nbc ;
  int bord ;
  
  //! Store the image (as a vector with intensity and gradient I, Ix, Iy) 
  int  firstTimeIn  ;
  //vpList< vpFeatureLine > Orilines;

 public:

  apOrientation *OriInfo;


  void buildFrom(vpMatrix &Ori, vpImage<vpRGBa> &Ic, vpMatrix &Zcoord, vpHomogeneousMatrix &cMo, vpImage<unsigned char> &Iv) ;
  void buildFrom(vpMatrix &Ori, vpImage<unsigned char> &Iv) ;

public: 

  void init() ;
  void init(int _nbr, int _nbc, double _Z) ;

  apFeatureGradientOrientation() ;
 
  //! Destructor.
  virtual ~apFeatureGradientOrientation()  ;

 public:
  vpCameraParameters cam ;
  void setCameraParameters(vpCameraParameters &_cam)  ;
  /*
    section Set/get Z
  */
  void update(int _nbr, int _nbc, double _Z);

  void set_Z(const double Z) ;
  double get_Z() const  ;


  /*
    vpBasicFeature method instantiation
  */
  vpList< vpFeatureLine > getFeatureLines();


  vpMatrix  interaction(const int select = FEATURE_ALL);
  vpMatrix interaction(unsigned int select = FEATURE_ALL) { return vpMatrix(); };
  void      interaction(vpMatrix &L, vpImage<unsigned char> &Iv);

  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;
  void error(const vpBasicFeature &s_star,
	     vpColVector &e)  ;

  void print(const int select = FEATURE_ALL ) const ;
  void print(unsigned int select = FEATURE_ALL) const{};

  apFeatureGradientOrientation *duplicate() const ;


  void display(const vpCameraParameters &cam,
	       vpImage<unsigned char> &I,
	       vpColor color=vpColor::green, unsigned int thickness=1) const ;
  void display(const vpCameraParameters &cam,
               vpImage<vpRGBa> &I,
               vpColor color=vpColor::green, unsigned int thickness=1) const ;
  void display(const vpCameraParameters&, const vpImage<unsigned char>&, const vpColor&, unsigned int) const {};
  void display(const vpCameraParameters&, const vpImage<vpRGBa>&, const vpColor&, unsigned int) const {};


  //! Compute the error between a visual features and zero
  vpColVector error(const int select = FEATURE_ALL)  ;

} ;



#endif
