/****************************************************************************
 *
 * $Id: vpMbTracker.cpp 2924 2010-11-16 17:34:36Z rtallonn $
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
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Generic model based tracker
 *
 * Authors:
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMbTracker.cpp
  \brief Generic model based tracker
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpColor.h>
#include <visp/vpIoTools.h>
#include <visp/vpException.h>
#include <visp/vpImageIo.h>
#include "vpMbTracker.h"
#include <visp/vpMatrixException.h>

#ifdef VISP_HAVE_COIN
//Inventor includes
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/misc/SoChildList.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoGetPrimitiveCountAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#endif


/*!
  Basic constructor.
  Set default values.

*/
vpMbTracker::vpMbTracker()
{
  modelInitialised = false;
  cameraInitialised = false;
  coinUsed = false;
}

/*!
  Basic destructor.

*/
vpMbTracker::~vpMbTracker()
{
#ifdef VISP_HAVE_COIN
  if(coinUsed){
    SoDB::finish();
    coinUsed = false;
  }
#endif
}


/*!
  Initialise the tracking by clicking on the image points corresponding to the 
  3D points (object frame) in the file _initFile. The structure of this file
  is (without the comments):
  \code
  4 // Number of points in the file (minimum is four)
  0.01 0.01 0.01    //  \
  ...               //  | coordinates in the object basis 
  0.01 -0.01 -0.01  // /
  \endcode

  \param _I : Input image
  \param _initFile : File containing the points where to click
  \param _displayHelp : Optionnal display of an image ( '_initFile.ppm' ). This 
    image may be used to show where to click. 
*/
void
vpMbTracker::initClick(const vpImage<unsigned char>& _I, const std::string& _initFile, const bool _displayHelp)
{
  vpHomogeneousMatrix last_cMo;
  vpPoseVector init_pos;

  // Load the last poses from files
  std::fstream finitpos ;
  std::fstream finit ;
  char s[FILENAME_MAX];

  sprintf(s, "%s.0.pos", _initFile.c_str());
  finitpos.open(s ,std::ios::in) ;
  if(finitpos.fail() ){
  	std::cout << "cannot read " << s << std::endl << "cMo set to identity" << std::endl;
  	last_cMo.setIdentity();
  }
  else{
    for (unsigned int i = 0; i < 6; i += 1){
      finitpos >> init_pos[i];
    }

    finitpos.close();
    last_cMo.buildFrom(init_pos) ;
  }
  std::cout <<"last_cMo : "<<std::endl << last_cMo <<std::endl;

  display(_I, last_cMo, cam, vpColor::green);
  vpDisplay::displayFrame(_I, last_cMo, cam, 0.05, vpColor::green);
  vpDisplay::flush(_I);

  std::cout << "No modification : left click " << std::endl;
  std::cout << "Modify initial pose : right click " << std::endl ;

  vpDisplay::displayCharString(_I, 15, 10,
			       "left click to validate, right click to modify initial pose",
			       vpColor::red);

  vpDisplay::flush(_I) ;

  vpImagePoint ip;
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  while (!vpDisplay::getClick(_I, ip, button)) ;


  if (button == vpMouseButton::button1){
    cMo = last_cMo ;
  }
  else
  {
    vpDisplay::display(_I) ;
    vpDisplay::flush(_I) ;

    vpPose pose ;

    pose.clearPoint() ;

    // file parser
    // number of points
    // X Y Z
    // X Y Z

    double X,Y,Z ;
    int i ;
    sprintf(s,"%s.init", _initFile.c_str());
    std::cout << "filename " << s << std::endl ;
    finit.open(s,std::ios::in) ;
    if (finit.fail())
    {
      std::cout << "cannot read " << s << std::endl;
	  throw vpException(vpException::ioError, "cannot read init file");
    }

    sprintf(s, "%s.ppm", _initFile.c_str());

    vpImage<vpRGBa> Iref ;
    //Display window creation and initialistation
#if defined VISP_HAVE_X11
    vpDisplayX d;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI d;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV d;
#endif
    try{
      if(_displayHelp){
        vpImageIo::readPPM(Iref,s) ;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
      d.init(Iref,10,500, "Where to initialize...")  ;
  	  vpDisplay::display(Iref) ;
  	  vpDisplay::flush(Iref);
#endif
	  	}
    }
    catch(...){}

    int n ;
    finit >> n ;
    std::cout << "number of points  " << n << std::endl ;
    vpPoint *P = new vpPoint [n]  ;
    for (i=0 ; i < n ; i++)
    {
      finit >> X ;
      finit >> Y ;
      finit >> Z ;
      P[i].setWorldCoordinates(X,Y,Z) ; // (X,Y,Z)
    }

    finit.close();

////////////////////////////////
    bool isWellInit = false;
    while(!isWellInit)
    {
////////////////////////////////
      for(int i=0 ; i< n ; i++)
      {
        std::cout << "Click on point " << i+1 << std::endl ;
        double x=0,y=0;
        vpDisplay::getClick(_I, ip) ;
        vpDisplay::displayCross(_I, ip, 5,vpColor::green) ;
        vpDisplay::flush(_I) ;
        vpPixelMeterConversion::convertPoint(cam, ip, x, y);
        P[i].set_x(x);
        P[i].set_y(y);

        std::cout << "click sur point " << ip << std::endl;

        vpDisplay::displayPoint (_I, ip, vpColor::green); //display target point
        pose.addPoint(P[i]) ; // and added to the pose computation point list
      }
      vpDisplay::flush(_I) ;

      vpHomogeneousMatrix cMo1, cMo2;
      pose.computePose(vpPose::LAGRANGE, cMo1) ;
      double d1 = pose.computeResidual(cMo1);
      pose.computePose(vpPose::DEMENTHON, cMo2) ;
      double d2 = pose.computeResidual(cMo2);

      if(d1 < d2){
        cMo = cMo1;
      }
      else{
        cMo = cMo2;
      }
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      std::cout << "cMo:" << std::endl << cMo << std::endl;

      display(_I, cMo, cam, vpColor::green);
      vpDisplay::displayCharString(_I, 15, 10,
				 "left click to validate, right click to re initialize object",
				 vpColor::red);

      vpDisplay::flush(_I) ;

      vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
      while (!vpDisplay::getClick(_I, ip, button)) ;


      if (button == vpMouseButton::button1)
      {
        isWellInit = true;
      }
      else
      {
        pose.clearPoint() ;
        vpDisplay::display(_I) ;
        vpDisplay::flush(_I) ;
      }
    }
////////////////////////////////////

    vpDisplay::displayFrame(_I, cMo, cam, 0.05, vpColor::red);

    delete [] P;

	//save the pose into file
//	sprintf(s,"%s.0.pos",filename);
  sprintf(s,"%s.0.pos", _initFile.c_str());
	finitpos.open(s, std::ios::out) ;
	init_pos.buildFrom(cMo);
	finitpos << init_pos;
	finitpos.close();
  }

  //save the pose into file
  sprintf(s, "%s.0.pos", _initFile.c_str());
  finitpos.open(s,std::ios::out) ;
  init_pos.buildFrom(cMo);
  finitpos << init_pos;
  finitpos.close();

  std::cout <<"cMo : "<<std::endl << cMo <<std::endl;

  init(_I, cMo);

}


/*!
  Compute \f$ J^T R \f$, with J the interaction matrix and R the vector of 
  residu.
  
  \throw vpMatrixException::incorrectMatrixSizeError if the sizes of the 
  matrices do not allow the computation.
  
  \warning The JTR matrix is resized.
  
  \param _interaction : The interaction matrix (size Nx6).
  \param _error : The residu vector (size Nx1).
  \param _JTR : The resulting JTR matrix (size 6x1).
  
*/
void 
vpMbTracker::computeJTR(const vpMatrix& _interaction, const vpColVector& _error, vpMatrix& _JTR)
{
  if(_interaction.getRows() != _error.getRows() || _interaction.getCols() != 6 ){
    throw vpMatrixException(vpMatrixException::incorrectMatrixSizeError, 
              "Incorrect matrices size in computeJTR.");
  }

  _JTR.resize(6, 1);
  const unsigned int N = _interaction.getRows();

  for (unsigned int i = 0; i < 6; i += 1){
    double ssum = 0;
    for (unsigned int j = 0; j < N; j += 1){
      ssum += _interaction[j][i] * _error[j];
    }
    _JTR[i][0] = ssum;
  }
}


