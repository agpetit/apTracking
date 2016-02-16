/****************************************************************************
*
* $Id $
*
* This software was developed at:
* INRIA Rennes Bretagne Atlantique
* Equipe Projet Lagadic
* Campus Universitaire de Beaulieu
* 35042 Rennes Cedex
* http://www.irisa.fr/lagadic
*
* This file is part of the Aronnax project.
*
* Copyright (C) 2009 Inria. All rights reserved.
* The content of this file cannot be modified, transmitted,
* licensed, transferred, sold or distributed by any means 
* without permission of INRIA.
*
* Authors:
* Laneurit Jean
*
*****************************************************************************/

#ifndef __VP_SL3_SE3_KALMAN__
#define __VP_SL3_SE3_KALMAN__

#include "vpSE3Kalman.h"
#include "visp/vpTranslationVector.h"
#include "visp/vpHomography.h"
#include "visp/vpGEMM.h"


/*!
\file vpSL3SE3Kalman.h

\brief Definition of the vpSL3Kalman class
*/

/*!
\class vpSL3SE3Kalman
\ingroup TrackingAlignment
\brief Definition of the vpSL3Kalman class
vpSL3SE3Kalman class provides a Kalman filter in SL3 group but estimate 
a camera position and rotation in SE3 + and camera kinematic torsor
If he is associated with a SL3 alignment minimization algorithm
The SL3 alignment minimization is initialized at each step 
and provides a observation for Kalman filter

\author Jean Laneurit (IRISA - INRIA Rennes)

\sa vpImage, vpImagePoint, vpMatrix, vpColVector, vpRowVector, vpTrianglePatch, vpPolygonPatch, vpWarp, vpAlignmentMinimization
*/

class vpSL3SE3Kalman : public vpSE3Kalman {
  
  private :
    
    
    /** Observations */
    /** homography observation */
    vpColVector _Xh;
    /** homography covariance */
    vpMatrix _Qh;
    /** jacobian for homography observation matrix */
    vpMatrix _Hh;
    /** Kalman gain matrix for homography information */
    vpMatrix _Kh;
    /** Innovation covariance for homography information */
    vpMatrix _IQh;
    /** Innovation vector for homography information */
    vpColVector _Ih;
    
    /** normalized homography  observation */
    vpColVector _Xhn;
    /** normalized homography  covariance */
    vpMatrix _Qhn;
    /** jacobian for normalized homography  observation matrix */
    vpMatrix _Hhn;
    /** Kalman gain matrix for normalized homography  information */
    vpMatrix _Khn;
    /** Innovation covariance for normalized homography  information */
    vpMatrix _IQhn;
    /** Innovation vector for normalized homography  information */
    vpColVector _Ihn;
    
    /** d Xv / d Ou*/
    vpSubMatrix _dH_dOu;
    /** d H / d P*/
    vpSubMatrix _dH_dP;
    /** Homography */
    vpColVector _H;
    vpColVector _Hn;
    vpMatrix _Hm;
    /** Normal Vector */
    vpColVector _N;
    vpRowVector _NT;
    /** Intrinsic parameters */
    vpMatrix _KKK;
    vpRowVector _K3;
    
    /** buffer */
    vpRowVector _buffer19;
    vpMatrix _buffer11;
    vpMatrix _buffer93c;
    vpMatrix _buffer129;
    vpMatrix _buffer128;
    vpMatrix _eye89;
    vpMatrix _buffer88;
    vpMatrix _buffer1212;
    vpMatrix _eye333;
    
    double _alpha;
    double _beta;
    
    /** compute alpha*/
    void ComputeAlpha();
    /** compute d H / d Ou*/
    void ComputeDH_DOu(const bool &);
    /** compute d H / d P*/
    void ComputeDH_DP(const bool &);
    
    void init(){};
    
  public :
    
    /** Default constructor */
    vpSL3SE3Kalman();
    /** Default constructor */
    ~vpSL3SE3Kalman(){};
    
    /** Vision update */    
    void setIntrinsics(const vpMatrix &K);
    void setNormal(const vpColVector &N);
    void init(const vpThetaUVector Ou, const vpTranslationVector &t); 
    vpHomography getHomography(){
	
        vpHomography _Hm;
	vpGEMM(_P,_NT,1.0,_ROu,1.0,_Hm);
	vpGEMM(_KKK,_Hm.stackColumns(),1,null,1.0,_H);
	_H.reshape(_Hm,3,3);
	return _Hm;
    }
    void setXh(const vpColVector &Xh);
    void setQh(const vpMatrix &Qh);
    void homographyUpdate();
    
    void normHomographyUpdate();
    void setXhn(const vpColVector &Xhn);
    void setQhn(const vpMatrix &Qhn);
    
};
#endif