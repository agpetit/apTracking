/*
 * apMHMeTracker.cpp
 *
 *  Created on: Jul 9, 2012
 *      Author: agpetit
 */

#include "apMHMeTracker.h"
#include <visp/vpDisplay.h>
#include <visp/vpColor.h>

#include <visp/vpTrackingException.h>
#include <visp/vpDebug.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 0




void
apMHMeTracker::init()
{
  if (DEBUG_LEVEL1)
    std::cout << "begin apMHMeTracker::init() " <<  std::endl ;

  vpTracker::init()  ;
  p.resize(2) ;
  selectDisplay = apMHMeSite::NONE ;

  if (DEBUG_LEVEL1)
    std::cout << "end apMHMeTracker::init() " <<  std::endl ;
}

apMHMeTracker::apMHMeTracker()
{
  if (DEBUG_LEVEL1)
    std::cout << "begin apMHMeTracker::apMHMeTracker() " <<  std::endl ;

  init();
  me = NULL ;
  display_point = false ;
  nGoodElement = 0;
  query_range = 0;
  seuil = 0;
  init_range = 1;

  if (DEBUG_LEVEL1)
    std::cout << "end apMHMeTracker::apMHMeTracker() " << std::endl ;
}

apMHMeTracker::apMHMeTracker(const apMHMeTracker& meTracker):vpTracker(meTracker)
{
  if (DEBUG_LEVEL1)
    std::cout << "begin apMHMeTracker::apMHMeTracker() " <<  std::endl ;

  init();

  me = meTracker.me;
  list = meTracker.list;
  nGoodElement = meTracker.nGoodElement;
  query_range = meTracker.query_range;
  init_range = meTracker.init_range;
  seuil = meTracker.seuil;
  display_point = meTracker.display_point;
  selectDisplay = meTracker.selectDisplay;
}

apMHMeTracker::~apMHMeTracker()
{
  if (DEBUG_LEVEL1) std::cout << "begin apMHMeTracker::~apMHMeTracker() " << std::endl ;

    list.clear();

  if (DEBUG_LEVEL1) std::cout << "end apMHMeTracker::~apMHMeTracker() " << std::endl ;
}

apMHMeTracker&
apMHMeTracker::operator = (apMHMeTracker& p)
{
  if (DEBUG_LEVEL1) std::cout << "begin apMHMeTracker::operator=" << std::endl ;

  list = p.list;
  me = p.me;
  selectDisplay = p.selectDisplay ;

  if (DEBUG_LEVEL1) std::cout << "end apMHMeTracker::operator=" << std::endl ;
  return *this;
}

int
apMHMeTracker::numberOfSignal()
{
  int number_signal=0;

  // Loop through all the points tracked from the contour
  for(int k = 0; k<list.size(); k++)
  {
    apMHMeSite P = list[k];
    if(P.suppress == 0) number_signal++;
  }

  return number_signal;
}

int
apMHMeTracker::totalNumberOfSignal()
{
  return list.size();

}

int
apMHMeTracker::outOfImage(int i, int j, int half, int rows, int cols)
{
  return (! ((i> half+2) &&
	     (i< rows -(half+2)) &&
	     (j>half+2) &&
	     (j<cols-(half+2)))
	  ) ;
}

int
apMHMeTracker::outOfImage(vpImagePoint iP, int half, int rows, int cols)
{
  int i = vpMath::round(iP.get_i());
  int j = vpMath::round(iP.get_j());
  return (! ((i> half+2) &&
	     (i< rows -(half+2)) &&
	     (j>half+2) &&
	     (j<cols-(half+2)))
	  ) ;
}


//! Virtual function that is called by lower classes apMHMeTrackerLine/Circle/Cylinder
void
apMHMeTracker::initTracking(const vpImage<unsigned char>& I)
{
  if (DEBUG_LEVEL1)
    std::cout << "begin apMHMeTracker::initTracking() " << std::endl ;


  // Must set range to 0
  int range_tmp = me->range;
  me->range=init_range;

  nGoodElement=0;

  int d = 0;
  vpImagePoint ip1, ip2;

  // Loop through list of sites to track
  for(int k = 0; k<list.size(); k++)
  {
	  apMHMeSite *refp = &(list[k]) ;//current reference pixel

    d++ ;
    // If element hasn't been suppressed
    if(refp->suppress==0)
    {
      try {
		//refp->trackMH(I,me,false);
    	  refp->track0(I,me,false);
      }
      catch(...)
      {
		// EM verifier quel signal est de sortie !!!
		vpERROR_TRACE("Error caught") ;
		throw ;
      }
      if(refp->suppress==0) nGoodElement++;
    }


    if(DEBUG_LEVEL2)
    {
      double a,b ;
      a = refp->i_1 - refp->i ;
      b = refp->j_1 - refp->j ;
      if(refp->suppress==0) {
		ip1.set_i( refp->i );
		ip1.set_j( refp->j );
		ip2.set_i( refp->i+a );
		ip2.set_j( refp->j+b );
		vpDisplay::displayArrow(I, ip1, ip2, vpColor::green) ;
      }
    }
    list[k] = *refp;
  }

  /*
  if (res != OK)
  {
    std::cout<< "In apMHMeTracker::initTracking(): " ;
    switch (res)
    {
    case  ERR_TRACKING:
      std::cout << "apMHMeTracker::initTracking:Track return ERR_TRACKING " << std::endl ;
      break ;
    case fatalError:
      std::cout << "apMHMeTracker::initTracking:Track return fatalError" << std::endl ;
      break ;
    default:
      std::cout << "apMHMeTracker::initTracking:Track return error " << res << std::endl ;
    }
    return res ;
  }
  */

  me->range=range_tmp;


  if (DEBUG_LEVEL1)
  std::cout << "end apMHMeTracker::initTracking() " << std::endl ;

}


void
apMHMeTracker::track(const vpImage<unsigned char>& I)
{
	  std::cout << list.size() << std::endl;
	if (DEBUG_LEVEL1)
    std::cout << "begin  apMHMeTracker::Track():" << std::endl ;
  if (list.size()==0)
  {
    if (DEBUG_LEVEL1)
    vpERROR_TRACE("Error Tracking: only %d "
		 "pixels when entered the function ",list.size()) ;
    throw(vpTrackingException(vpTrackingException::notEnoughPointError,
			      "too few pixel to track")) ;

  }

  vpImagePoint ip1, ip2;
  nGoodElement=0;
  //  int d =0;
  // Loop through list of sites to track
  for(int k = 0; k<list.size(); k++)
  {
	 apMHMeSite *s = &(list[k]) ;//current reference pixel

    //    d++ ;
    // If element hasn't been suppressed
    if(s->suppress==0)
    {

      try{
	//	vpERROR_TRACE("%d",d ) ;
	//	vpERROR_TRACE("range %d",me->range) ;
	 //s->trackMH(I,me,false);
    	  s->track0(I,me,true);
      }
      catch(vpTrackingException)
      {
	vpERROR_TRACE("catch exception ") ;
	s->suppress=2 ;
      }
      if(s->suppress != 2)
      {
	nGoodElement++;
	//if(DEBUG_LEVEL2)
	{
	  double a,b;
	  a = s->i_1 - s->i ;
	  b = s->j_1 - s->j ;
	  //if(s->suppress==0)
	  {
	    /*ip1.set_i( s->i );
	    ip1.set_j( s->j );
	    ip2.set_i( s->i+a*5 );
	    ip2.set_j( s->j+b*5 );*/
	    ip1.set_i( s->i_1 );
	    ip1.set_j( s->j_1 );
	    ip2.set_i( s->i );
	    ip2.set_j( s->j );
	    //vpDisplay::displayArrow(I, ip1, ip2, vpColor::yellow, 10);
	  }
	}

      }
      list[k] = *s;
    }
  }

  if (DEBUG_LEVEL1)
    std::cout << "end  apMHMeTracker::Track()" <<nGoodElement << std::endl ;

}

void
apMHMeTracker::initTrackingG(const vpImage<unsigned char>& I, const vpImage<unsigned char>& gradMap)
{
  if (DEBUG_LEVEL1)
    std::cout << "begin apMHMeTracker::initTracking() " << std::endl ;


  // Must set range to 0
  int range_tmp = me->range;
  me->range=init_range;

  nGoodElement=0;

  int d = 0;
  vpImagePoint ip1, ip2;

  // Loop through list of sites to track
  for(int k = 0; k<list.size(); k++)
  {
	  apMHMeSite *refp = &(list[k]) ;//current reference pixel

    d++ ;
    // If element hasn't been suppressed
    if(refp->suppress==0)
    {
      try {
		refp->trackMHGrad(I,gradMap, me, false);
		//refp->track0(I, me,false);
      }
      catch(...)
      {
		// EM verifier quel signal est de sortie !!!
		vpERROR_TRACE("Error caught") ;
		throw ;
      }
      if(refp->suppress==0) nGoodElement++;
    }


    if(DEBUG_LEVEL2)
    {
      double a,b ;
      a = refp->i_1 - refp->i ;
      b = refp->j_1 - refp->j ;
      if(refp->suppress==0) {
		ip1.set_i( refp->i );
		ip1.set_j( refp->j );
		ip2.set_i( refp->i+a );
		ip2.set_j( refp->j+b );
		vpDisplay::displayArrow(I, ip1, ip2, vpColor::green) ;
      }
    }
    list[k] = *refp;
  }

  /*
  if (res != OK)
  {
    std::cout<< "In apMHMeTracker::initTracking(): " ;
    switch (res)
    {
    case  ERR_TRACKING:
      std::cout << "apMHMeTracker::initTracking:Track return ERR_TRACKING " << std::endl ;
      break ;
    case fatalError:
      std::cout << "apMHMeTracker::initTracking:Track return fatalError" << std::endl ;
      break ;
    default:
      std::cout << "apMHMeTracker::initTracking:Track return error " << res << std::endl ;
    }
    return res ;
  }
  */

  me->range=range_tmp;


  if (DEBUG_LEVEL1)
  std::cout << "end apMHMeTracker::initTracking() " << std::endl ;

}


void
apMHMeTracker::trackG(const vpImage<unsigned char>& I, const vpImage<unsigned char>& gradMap)
{
	if (DEBUG_LEVEL1)
    std::cout << "begin  apMHMeTracker::Track():" << std::endl ;
  if (list.size()==0)
  {
    if (DEBUG_LEVEL1)
    vpERROR_TRACE("Error Tracking: only %d "
		 "pixels when entered the function ",list.size()) ;
    throw(vpTrackingException(vpTrackingException::notEnoughPointError,
			      "too few pixel to track")) ;

  }

  vpImagePoint ip1, ip2;
  nGoodElement=0;
  //  int d =0;
  // Loop through list of sites to track
  for(int k = 0; k<list.size(); k++)
  {
	 apMHMeSite *s = &(list[k]) ;//current reference pixel

    //    d++ ;
    // If element hasn't been suppressed
    if(s->suppress==0)
    {

      try{
	//	vpERROR_TRACE("%d",d ) ;
	//	vpERROR_TRACE("range %d",me->range) ;
	  s->trackMHGrad(I,gradMap,me,false);
    	  //s->track0(I,me,true);
     //std::cout << " cand " << s->candidateList[0].i << std::endl;
      }
      catch(vpTrackingException)
      {
	vpERROR_TRACE("catch exception ") ;
	s->suppress=2 ;
      }

      if(s->suppress != 2)
      {
	nGoodElement++;

	//if(DEBUG_LEVEL2)
	{
	  double a,b;
	  a = s->i_1 - s->i ;
	  b = s->j_1 - s->j ;
	  //if(s->suppress==0)
	  {
	    /*ip1.set_i( s->i );
	    ip1.set_j( s->j );
	    ip2.set_i( s->i+a*5 );
	    ip2.set_j( s->j+b*5 );*/
	    ip1.set_i( s->i_1 );
	    ip1.set_j( s->j_1 );
	    ip2.set_i( s->i );
	    ip2.set_j( s->j );
	    //vpDisplay::displayArrow(I, ip1, ip2, vpColor::yellow, 10);
	  }
	}

      }
      list[k] = *s;
    }
  }

  if (DEBUG_LEVEL1)
    std::cout << "end  apMHMeTracker::Track()" <<nGoodElement << std::endl ;

}


void
apMHMeTracker::display(const vpImage<unsigned char>& I)
{
  if (DEBUG_LEVEL1)
  {
    std::cout <<"begin apMHMeTracker::displayList() " << std::endl ;
    std::cout<<" There are "<<list.size()<< " sites in the list " << std::endl ;
  }
  vpImagePoint ip;
  for(int k = 0; k<list.size(); k++)
  {
	  apMHMeSite p = list[k] ;

    if(p.suppress == 1) {
      ip.set_i( p.i );
      ip.set_j( p.j);
      vpDisplay::displayCross(I, ip, 2, vpColor::white) ; // Contrast
    }
    else if(p.suppress == 2) {
      ip.set_i( p.i );
      ip.set_j( p.j);
      vpDisplay::displayCross(I, ip, 2,vpColor::blue) ; // Threshold
    }
    else if(p.suppress == 3) {
      ip.set_i( p.i );
      ip.set_j( p.j);
      vpDisplay::displayCross(I, ip, 3, vpColor::green) ; // M-estimator
    }
    else if(p.suppress == 0) {
      ip.set_i( p.i );
      ip.set_j( p.j);
      vpDisplay::displayCross(I, ip, 2, vpColor::red) ; // OK
    }

  }
  if (DEBUG_LEVEL1)
  {
    std::cout <<"end apMHMeTracker::displayList() " << std::endl ;
  }
}


void
apMHMeTracker::display(const vpImage<unsigned char>& I,vpColVector &w,int &index_w)
{

  for(int k = 0; k<list.size(); k++)
  {
	 apMHMeSite *P = &(list[k]);

    if(P->suppress == 0)
    {
      P->weight = w[index_w];
      index_w++;
    }
    list[k] = *P;

  }
  display(I);
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
#undef DEBUG_LEVEL3

