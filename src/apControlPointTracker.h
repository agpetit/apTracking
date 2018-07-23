/*
 * apControlPointTracker.h
 *
 *  Created on: Jan 24, 2011
 *      Author: agpetit
 */
#ifndef apControlPointTracker_HH
#define apControlPointTracker_HH


#include <math.h>
#include <vector>
#include <iostream>

#include <visp/vpConfig.h>
#include <visp/vpColVector.h>
#include "vpPointSite.h"
#include <visp/vpMe.h>
//#include <visp/vpList.h>
#include <visp/vpTracker.h>


class apControlPointTracker : public vpTracker
{
public:

  //! Tracking dependent variables/functions =====================

  //! List of tracked points
  vpPointSite s ;
std::vector<vpPointSite> candidates;
  //vpPointSite* candidateVector;
  //! Ecm initialisation parameters
  vpMe *me ;
  //! Used for backwards compatibility...could be removed
  int nGoodElement;
  int query_range;
  int init_range;
  double seuil;
  bool display_point;// if 1 (TRUE) displays the line that is being tracked
  bool isvalid;

  //! Distance variables/functions ==========================================

  //! Constructor/Destructor
  apControlPointTracker() ;
  apControlPointTracker(const apControlPointTracker& meTracker) ;
  virtual ~apControlPointTracker() ;
  void init() ;

  apControlPointTracker& operator =(apControlPointTracker& f);

  //! Displays the number of elements in the list
  void displayNumberOfElements() ;
  void setMe(vpMe *me1) { me = me1 ; }
  int outOfImage( int i , int j , int half , int rows , int cols) ;
  int outOfImage( vpImagePoint iP , int half , int rows , int cols) ;

  void isValid() ;

  //! Virtual functions for vpMeTracker
  //! Feature dependent functions

  //!display contour
  virtual void display(const vpImage<unsigned char> &I, vpColor col)=0;
  //!Sample pixels at a given interval
  virtual void sample(const vpImage<unsigned char> &image)=0;

  void initTracking(const vpImage<unsigned char>& I);
  //!Track sampled pixels
  void track(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec);

  void trackPred(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec);
  //!Displays the status of me site

  void trackMH(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec);

  void display(const vpImage<unsigned char>& I);
  //!Displays the status of me sites
  void display(const vpImage<unsigned char>& I,vpColVector &w,int &index_w);
  void setDisplay(vpPointSite::vpPointSiteDisplayType select)  {
    selectDisplay = select ;
  }

protected:
  vpPointSite::vpPointSiteDisplayType selectDisplay ;

};


#endif


