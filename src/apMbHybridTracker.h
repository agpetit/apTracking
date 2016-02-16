#ifndef apMbHybridTracker_h
#define apMbHybridTracker_h

#include <visp/vpConfig.h>
#include <visp/vpMbTracker.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include "vpPointsTracker.h"
#include <visp/vpPoint.h>
#include <visp/vpMe.h>
#include <visp/vpFeatureLuminance.h>
#include "vpMbtControlPoint.h"
#include "apHoughVote.h"
//#include "vpAROgre.h"
#include "apOgre.h"
#include "vpMbPointsTracker.h"
#include "apImageFilter.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <string.h>


#ifdef VISP_HAVE_OPENCV
#  if VISP_HAVE_OPENCV_VERSION >= 0x020101
#    include <opencv2/core/core.hpp>
#    include <opencv2/imgproc/imgproc.hpp>
#    include <opencv2/imgproc/imgproc_c.h>
#  else
#    include <cv.h>
#  endif
#endif

using namespace cv;

/*!
  \class vpMbEdgeTracker
  \ingroup ModelBasedTracking 
  \brief Make the complete tracking of an object by using its CAD model.
*/

class VISP_EXPORT apMbHybridTracker: public vpMbPointsTracker
{
  protected :

    //apOgre ogre;
    vpImage<vpRGBa> Inormd;
    vpImage<unsigned char> Ior;

    int iinf,isup,jinf,jsup;
  
 public:
  
  apMbHybridTracker(int width, int heigth);
  virtual ~apMbHybridTracker();
  
  void trackHyb(const vpImage<unsigned char>& I, const double dist, apOgre &_ogre);
  //void loadOgre(apOgre &_ogre);

 protected:
  void updateRT(apOgre &_ogre);
  void cast();
  void computeVVSHyb(const vpImage<unsigned char>& _I, apOgre &_ogre);


};

#endif

