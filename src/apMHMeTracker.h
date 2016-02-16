/*
 * apMHMeTracker.h
 *
 *  Created on: Jul 9, 2012
 *      Author: agpetit
 */

#ifndef APMHMETRACKER_H_
#define APMHMETRACKER_H_

#include <math.h>
#include <iostream>

#include <visp/vpConfig.h>
#include <visp/vpColVector.h>
#include <visp/vpMeSite.h>
#include <visp/vpMe.h>
#include <visp/vpTracker.h>
#include "apMHMeSite.h"


class VISP_EXPORT apMHMeTracker : public vpTracker
{
protected:

  //! Tracking dependent variables/functions =====================

  //! List of tracked points
  std::vector<apMHMeSite> list ;
  //! Ecm initialisation parameters
  vpMe *me ;
  //! Used for backwards compatibility...could be removed
  int nGoodElement;
  int query_range;
  int init_range;
  double seuil;
  bool display_point;// if 1 (TRUE) displays the line that is being tracked

  //! Distance variables/functions ==========================================

  //! Constructor/Destructor
  apMHMeTracker();
  apMHMeTracker(const apMHMeTracker& meTracker) ;
  virtual ~apMHMeTracker() ;
  void init() ;

  apMHMeTracker& operator =(apMHMeTracker& f);

  //! Displays the number of elements in the list
  void displayNumberOfElements() ;
  void setMe(vpMe *me1) { me = me1 ; }
  //void setCameraParameters(vpCameraParameters _cam){ cam = _cam;}
  int outOfImage( int i , int j , int half , int rows , int cols) ;
  int outOfImage( vpImagePoint iP , int half , int rows , int cols) ;

  int numberOfSignal() ;
  int totalNumberOfSignal() ;

  //! Virtual functions for vpMeTracker
  //! Feature dependent functions

  //!display contour
  virtual void display(const vpImage<unsigned char> &I, vpColor col)=0;
  //!Sample pixels at a given interval
  virtual void sample(const vpImage<unsigned char> &image)=0;

  void initTracking(const vpImage<unsigned char>& I);
  void initTrackingG(const vpImage<unsigned char>& I, const vpImage<unsigned char>& gradMap);
  //!Track sampled pixels
  void track(const vpImage<unsigned char>& I);
  //!Displays the status of me site
  void trackG(const vpImage<unsigned char>& I, const vpImage<unsigned char>& gradMap);
  void display(const vpImage<unsigned char>& I);
  //!Displays the status of me sites
  void display(const vpImage<unsigned char>& I,vpColVector &w,int &index_w);
  void setDisplay(apMHMeSite::vpMeSiteDisplayType select)  {
    selectDisplay = select ;
  }

protected:
  apMHMeSite::vpMeSiteDisplayType selectDisplay ;

};


#endif
