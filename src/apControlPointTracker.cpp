/*!
  \file apControlPointTracker.cpp
  \brief Contains abstract elements for a Distance to Feature type feature.
*/

#include "apControlPointTracker.h"
#include <visp/vpDisplay.h>
#include <visp/vpColor.h>
#include <visp/vpImageIo.h>

#include <visp/vpTrackingException.h>
#include <visp/vpDebug.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 0




void
apControlPointTracker::init()
{
  if (DEBUG_LEVEL1)
    std::cout << "begin apControlPointTracker::init() " <<  std::endl ;

  vpTracker::init()  ;
  p.resize(2) ;
  selectDisplay = vpPointSite::NONE ;

  if (DEBUG_LEVEL1)
    std::cout << "end apControlPointTracker::init() " <<  std::endl ;
}

apControlPointTracker::apControlPointTracker()
{
  if (DEBUG_LEVEL1)
    std::cout << "begin apControlPointTracker::apControlPointTracker() " <<  std::endl ;

  init();
  me = NULL ;
  display_point = true ;
  nGoodElement = 0;
  query_range = 0;
  seuil = 0;
  init_range = 1;
  candidates.resize(1);

  if (DEBUG_LEVEL1)
    std::cout << "end apControlPointTracker::apControlPointTracker() " << std::endl ;
}

apControlPointTracker::apControlPointTracker(const apControlPointTracker& meTracker):vpTracker(meTracker)
{
  if (DEBUG_LEVEL1)
    std::cout << "begin apControlPointTracker::apControlPointTracker() " <<  std::endl ;

  init();
  
  me = meTracker.me;
  s = meTracker.s;
  nGoodElement = meTracker.nGoodElement;
  query_range = meTracker.query_range;
  init_range = meTracker.init_range;
  seuil = meTracker.seuil;
  display_point = meTracker.display_point;
  selectDisplay = meTracker.selectDisplay;
}

apControlPointTracker::~apControlPointTracker()
{

}

apControlPointTracker&
apControlPointTracker::operator = (apControlPointTracker& p)
{
  if (DEBUG_LEVEL1) std::cout << "begin apControlPointTracker::operator=" << std::endl ;

  s = p.s;
  me = p.me;
  selectDisplay = p.selectDisplay ;

  if (DEBUG_LEVEL1) std::cout << "end apControlPointTracker::operator=" << std::endl ;
  return *this;
}

void
apControlPointTracker::isValid()
{
    if(s.suppress == 0) isvalid = true;
    else isvalid = false;
}


int
apControlPointTracker::outOfImage(int i, int j, int half, int rows, int cols)
{
  return (! ((i> half+2) &&
	     (i< rows -(half+2)) &&
	     (j>half+2) &&
	     (j<cols-(half+2)))
	  ) ;
}

int
apControlPointTracker::outOfImage(vpImagePoint iP, int half, int rows, int cols)
{
  int i = vpMath::round(iP.get_i());
  int j = vpMath::round(iP.get_j());
  return (! ((i> half+2) &&
	     (i< rows -(half+2)) &&
	     (j>half+2) &&
	     (j<cols-(half+2)))
	  ) ;
}


//! Virtual function that is called by lower classes apControlPointTrackerLine/Circle/Cylinder
void
apControlPointTracker::initTracking(const vpImage<unsigned char>& I)
{
  if (DEBUG_LEVEL1)
    std::cout << "begin apControlPointTracker::initTracking() " << std::endl ;


  // Must set range to 0
  int range_tmp = me->range;
  me->range=init_range;

  nGoodElement=0;

  int d = 0;
  vpImagePoint ip1, ip2;
  
  // Loop through list of sites to track
  //list.front();
  //while(!list.outside())

    // If element hasn't been suppressed
    if(s.suppress==0)
    {
      try {
		s.track(I,me,false);
      }
      catch(...)
      {
		// EM verifier quel signal est de sortie !!!
		vpERROR_TRACE("Error caught") ;
		throw ;
      }
      if(s.suppress==0) nGoodElement++;
    }


    if(DEBUG_LEVEL2)
    {
      double a,b ;
      a = s.i_1 - s.i ;
      b = s.j_1 - s.j ;
      if(s.suppress==0) {
		ip1.set_i( s.i );
		ip1.set_j( s.j );
		ip2.set_i( s.i+a );
		ip2.set_j( s.j+b );
		//vpDisplay::displayArrow(I, ip1, ip2, vpColor::green) ;
      }
    }

  me->range=range_tmp;


  if (DEBUG_LEVEL1)
  std::cout << "end apControlPointTracker::initTracking() " << std::endl ;

}


void
apControlPointTracker::track(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec)
{
  if (DEBUG_LEVEL1)
    std::cout << "begin  apControlPointTracker::Track():" << std::endl ;


  vpImagePoint ip1, ip2;

    if(s.suppress==0)
    {

      try{
	//	vpERROR_TRACE("%d",d ) ;
	//	vpERROR_TRACE("range %d",me->range) ;
    	  if (s.convlt==0)
    	  {
	       s.track(I,me,false);
    		  //s.track2(I,Iprec,me,false);
    		  //s.trackProfile(I,Iprec,me);
    	  }
    	  else
    	  {
    	  s.track(I,me,false);
    			 //s.trackProfile(I,Iprec,me);
    		  //s.track2(I,Iprec,me,false);
    	  }

	  double a,b ;
	  a = s.i_1 - s.i ;
	  b = s.j_1 - s.j ;
	  if(s.suppress==0) {
			if(DEBUG_LEVEL2)
			{
	    ip1.set_i( s.i_1 );
	    ip1.set_j( s.j_1 );
	    //ip2.set_i( s.i_1-a*5 );
	    //ip2.set_j( s.j_1-b*5 );
	    ip2.set_i( s.i);
	    ip2.set_j( s.j);
        vpDisplay::displayArrow(I, ip1, ip2, vpColor::red,1) ;
        //vpDisplay::displayCross(I, ip1,2, vpColor::red, 2) ;
        //vpDisplay::displayCross(I, ip2,2, vpColor::white, 2) ;
			}
      }

      }
      catch(vpTrackingException)
      {
	vpERROR_TRACE("catch exception ") ;
	s.suppress=2 ;
      }

    }


  if (DEBUG_LEVEL1)
    std::cout << "end  apControlPointTracker::Track()" <<nGoodElement << std::endl ;

}


void
apControlPointTracker::trackMH(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec)
{
  if (DEBUG_LEVEL1)
    std::cout << "begin  apControlPointTracker::Track():" << std::endl ;

  vpImagePoint ip1, ip2;

    // If element hasn't been suppressed
    if(s.suppress==0)
    {

      try{
	//	vpERROR_TRACE("%d",d ) ;
	//	vpERROR_TRACE("range %d",me->range) ;
    	  if (s.convlt==0)
    	  {
           s.trackMH(I,me,false);
    	 //s.track2(I,Iprec,me,false);
         //s.trackProfile(I,Iprec,me);
	     //std::cout << " suppress " << s.suppress << std::endl;

    	  }
    	  else
    	  {
           s.trackMH(I,me,false);
    	   //s.track2(I,Iprec,me,false);
           //s.trackProfile(I,Iprec,me);
    	  }
	 //std::cout<< " sj "<<s.j <<" si "<<s.i <<std::endl;
	  double a,b ;
	  a = s.i_1 - s.i ;
	  b = s.j_1 - s.j ;
	  if(s.suppress==0) {
			if(DEBUG_LEVEL2)
			{
	    ip1.set_i( s.i_1 );
	    ip1.set_j( s.j_1 );
	    //ip2.set_i( s.i_1-a*5 );
	    //ip2.set_j( s.j_1-b*5 );
	    ip2.set_i( s.i);
	    ip2.set_j( s.j);
        //vpDisplay::displayArrow(I, ip1, ip2, vpColor::red,1) ;
//        vpDisplay::displayCross(I, ip1,2, vpColor::green, 2) ;
//        vpDisplay::displayCross(I, ip2,2, vpColor::white, 2) ;
			}
	  }



      }
      catch(vpTrackingException)
      {
	vpERROR_TRACE("catch exception ") ;
	s.suppress=2 ;
      }

      for (int m=0; m<s.getNumCandidates();m++)
      {
      candidates.push_back(s.candidateList[m]);
      }
      //std::cout<< " suppress " << s.suppress<<std::endl;
    }

  if (DEBUG_LEVEL1)
    std::cout << "end  apControlPointTracker::Track()" <<nGoodElement << std::endl ;

}



void
apControlPointTracker::trackPred(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec)
{
  if (DEBUG_LEVEL1)
    std::cout << "begin  apControlPointTracker::Track():" << std::endl ;

  vpImagePoint ip1, ip2;

    // If element hasn't been suppressed
    if(s.suppress==0)
    {

      try{
	//	vpERROR_TRACE("%d",d ) ;
	//	vpERROR_TRACE("range %d",me->range) ;
    	  if (s.convlt==0)
    	  {
	 //s.track(I,me,false);
    		  s.track2(I,Iprec,me,false);
    	  }
    	  else
    	  {
    			// s.track(I,me,false);
    		  s.track2(I,Iprec,me,false);
    			 //std::cout<<" ok1"<<std::endl;
    	  }
	 //std::cout<< " sj "<<s.j <<" si "<<s.i <<std::endl;

	  double a,b ;
	  a = s.i_1 - s.i ;
	  b = s.j_1 - s.j ;
	  if(s.suppress==0) {
			if(DEBUG_LEVEL2)
			{
	    ip1.set_i( s.i_1 );
	    ip1.set_j( s.j_1 );
	    //ip2.set_i( s.i_1-a*5 );
	    //ip2.set_j( s.j_1-b*5 );
	    ip2.set_i( s.i);
	    ip2.set_j( s.j);
	    //vpDisplay::displayArrow(I, ip1, ip2, vpColor::green,1) ;
	    //vpDisplay::displayCross(I, ip1,2, vpColor::green, 2) ;
	    //vpDisplay::displayCross(I, ip2,2, vpColor::white, 2) ;
			}
	  }



      }
      catch(vpTrackingException)
      {
	vpERROR_TRACE("catch exception ") ;
	s.suppress=2 ;
      }
    }


  if (DEBUG_LEVEL1)
    std::cout << "end  apControlPointTracker::Track()" <<nGoodElement << std::endl ;

}


void
apControlPointTracker::display(const vpImage<unsigned char>& I)
{
  if (DEBUG_LEVEL1)
  {
    std::cout <<"begin apControlPointTracker::displayList() " << std::endl ;
  }
  vpImagePoint ip;

    if(s.suppress == 1) {
      ip.set_i( s.i );
      ip.set_j( s.j);
      vpDisplay::displayCross(I, ip, 2, vpColor::white) ; // Contrast
    }
    else if(s.suppress == 2) {
      ip.set_i( s.i );
      ip.set_j( s.j);
      vpDisplay::displayCross(I, ip, 2,vpColor::blue) ; // Threshold
    }
    else if(s.suppress == 3) {
      ip.set_i( s.i );
      ip.set_j( s.j);
      vpDisplay::displayCross(I, ip, 3, vpColor::green) ; // M-estimator
    }
    else if(s.suppress == 0) {
      ip.set_i( s.i );
      ip.set_j( s.j);
      vpDisplay::displayCross(I, ip, 2, vpColor::red) ; // OK
    }


  if (DEBUG_LEVEL1)
  {
    std::cout <<"end apControlPointTracker::displayList() " << std::endl ;
  }
}


void
apControlPointTracker::display(const vpImage<unsigned char>& I,vpColVector &w,int &index_w)
{

    if(s.suppress == 0)
    {
      s.weight = w[index_w];
      index_w++;
    }
  display(I);
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
#undef DEBUG_LEVEL3

