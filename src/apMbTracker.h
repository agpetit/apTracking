/*
 * apMbTracker.h
 *
 *  Created on: March 10, 2011
 *      Author: Antoine Petit
 */

#ifndef apMbTracker_h
#define apMbTracker_h

#include <visp/vpKltOpencv.h>
#include <visp/vpConfig.h>
#include <visp/vpImageFilter.h>
#include <visp/vpMbTracker.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include "apControlPointTracker.h"
#include <visp/vpPoint.h>
#include <visp/vpMe.h>
#include <visp/vpFeatureLuminance.h>
#include <visp/vpImageTools.h>
#include "apControlPoint.h"
#include "apHoughVote.h"
#include "apRend.h"
#include "apMbtDistanceLineMH.h"
#include "apImageFilter.h"
#include "apKalmanFilter.h"
#include "apCCDTracker.h"
#include "apKltControlPoint.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <string.h>

#if defined(VISP_HAVE_COIN)
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

#ifdef VISP_HAVE_OPENCV
#  if VISP_HAVE_OPENCV_VERSION >= 0x020101
#    include <opencv2/core/core.hpp>
#    include <opencv2/imgproc/imgproc.hpp>
#    include <opencv2/imgproc/imgproc_c.h>
#  else
#    include <cv.h>
#  endif
#endif

#include "apDetection.h"
#include "apLearn.h"
#include "apSegmentation.h"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>
#include "serialization.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


#include "structures.h"

#include <zmq.hpp>
using namespace cv;

struct apKLTTrackerParameters
{
  apKLTTrackerParameters(): blocksize(12), npoints(500), history(25), windowsize(12), quality(0.000001), mindist(6), harrisfree(0.04), useharris(1), pyramidlevels(1)
  {
  }
  apKLTTrackerParameters(int p1,
            int p2,
            int p3,
            int p4,
            double p5,
            int p6,
            double p7,
            int p8,
            int p9
            )
  {
    blocksize = p1;
    npoints = p2;
    history = p3;
    windowsize = p4;
    quality = p5;
    mindist = p6;
    harrisfree = p7;
    useharris = p8;
    pyramidlevels = p9;
  }

  ~apKLTTrackerParameters()
  {
  }
  int blocksize;
  int npoints;
  int history;
  int windowsize;
  double quality;
  int mindist;
  double harrisfree;
  int useharris;
  int pyramidlevels;
};



/*!
  \class vpMbEdgeTracker
  \ingroup ModelBasedTracking 
  \brief Make the complete tracking of an object by using its CAD model.
*/

class apMbTracker: public vpMbTracker, public apControlPointTracker
{
  public :
	  typedef enum
	    {
	      POINTS_SH,
	      POINTS_MH,
	      LINES_MH,
	      CCD_SH,
	      CCD_MH,
	      CCD_LINES_MH,
	      CCD_MH_KLT,
	      CCD_LINES_MH_KLT
	    } apTrackingType;

  protected :
    
    /*! If this flag is true, the interaction matrix
     extracted from the FeatureSet is computed at each
     iteration in the visual servoing loop.
    */
    int compute_interaction;

    //! Tracking type
    apTrackingType trackingType;

    //! The gain of the virtual visual servoing stage. 
    double lambda;

    //! The moving edges parameters. 
    vpMe  me;

    //! The 3D rendering parameters used by the tracker and OpenSceneGraph.
    apRend rendparam;

    double zn, zf;

    //! Detection parameters
    apDetection detect;

    //! Segmentation parameters
    apSegmentationParameters seg;

    //! Learning parameters
    apLearn learn;
    //! Vector of list of all the lines tracked (each line is linked to a list of moving edges). Each element of the vector is for a scale (element 0 = level 0 = no subsampling).
    std::vector< std::vector< apControlPoint*> > points;

    std::vector< std::vector< apMbtDistanceLineMH*> > lines;
    //std::vector< std::vector< apMbtDistanceLineMH*> > linesTemp;
    std::vector< std::vector< apMbtDistanceLineMH*> > lines_cand;

    std::vector< std::map< int, apMbtDistanceLineMH> > newLinesLm;
    std::vector< std::map< int, apMbtDistanceLineMH> > linesLm;

    vpKltOpencv kltTracker;

    std::vector<std::map<int, apKltControlPoint> > kltPoints;
    std::vector<std::map<int, int> > kltPointsInd;

    std::vector<vpHomogeneousMatrix> pose_vect;
    int nbdraw;

    //! Features luminance (desired and current) computed on image gradient norms, in case of dense tracking with virtual visual servoing based on image gradients.
    vpFeatureLuminance sId;
    vpFeatureLuminance sI;

    //std::vector< vpList< apControlPoint*> > points_1;
    //! Number of control points.
    int npoints;
    int npointssil;
    int nline ;
    int nkltPoints;
    int nkltPointsO;

    apHoughVote V;
    vpColVector Theta;
    vpColVector Rho;
    
    //! If true, the control points are displayed during the track() method.
    //bool displayMe;
    
    //! Percentage of good points over total number of points below which tracking is supposed to have failed.
    double percentageGdPt;
    
    //! Vector of scale level to use for the multi-scale tracking.
    std::vector<bool> scales;
    //! Previous image
    vpImage<unsigned char> Iprec;
    vpImage<vpRGBa> IprecRGB;
    //! Image gradients
    vpImage<unsigned char> gradMap;
    //! Pyramid of image associated to the current image. This pyramid is computed in the init() and in the track() methods.
    std::vector< const vpImage<unsigned char>* > Ipyramid;
    //! Pyramid of image associated to the previous. This pyramid is computed in the init() and in the track() methods.
    std::vector< const vpImage<unsigned char>* > Ipyramidprec;

    std::vector< const vpImage<vpRGBa>* > IRGBpyramid;
    //! Current scale level used. This attribute must not be modified outsied of the downScale() and upScale() methods, as it used to specify to some methods which set of distanceLine use.

    std::vector<vpRobust*> robustlines;


    unsigned int scaleLevel;

  	std::vector<double> residus;//residus associés aux différentes poses obtenues
  	double rmin;

  	bool computeCovariance;

  	bool useKalman;

  	bool useRGB;

  	apKalmanParameters kalmanParameters;

  	apCCDTracker CCDTracker;

  	apCCDParameters CCDParameters;

  	apKLTTrackerParameters KLTTrackerParameters;

  	double weight_p;
  	double weight_me;
  	double weight_ccd;
  	double weight_klt;

  	int frame;
  	int frame0;

	int firstFrame;
	vpMatrix truePose;

	double meantime;

	vpHomogeneousMatrix cMoV;

	vpHomogeneousMatrix c0Mo;
	vpHomogeneousMatrix ctTc0;


        int fixedrotationx, fixedrotationy, fixedrotationz;

        std::vector<point3d> controlpoints;

private:

    zmq::context_t     m_context{1};
    zmq::socket_t      *m_socketSub;
    zmq::socket_t      *m_socketPub;
  
 public:

    //Covariance matrix
    vpMatrix covarianceMatrix;
    vpMatrix covarianceMatrixME;
    vpMatrix covarianceMatrixP;
    vpMatrix covarianceMatrixL;
    vpMatrix covarianceMatrixCCD;
    vpMatrix covarianceMatrixKLT;

    bool predictKLT;

    vpHomogeneousMatrix cMoprec;
    vpHomogeneousMatrix opMo;
    vpHomogeneousMatrix oMct;


    vpImage<vpRGBa> Inormdprec;
    vpImage<unsigned char> Iorprec;
    vpImage<unsigned char> Itexprec;
    vpImage<unsigned char> IdN;
    vpImage<unsigned char> IdiffI;

    double timeextract;
    double timeextractklt;
    double timetrack;
    double timetrackklt;
    double timevvs;

    double sigmag;
    double sigmap;

            int itert;


  
  apMbTracker();
  virtual ~apMbTracker();
  
  inline void setPose(const vpHomogeneousMatrix &cMo) {this->cMo = cMo;}
  inline void setPose(const vpImage<unsigned char>& I, const vpHomogeneousMatrix& cMo){};
  
  /*!
    Set the value of the gain used to compute the control law.
    \param lambda : the desired value for the gain.
  */
  inline void setLambda(const double lambda) {this->lambda = lambda;}
  
  void setMovingEdge(const vpMe &_me);
  void setRendParameters(const apRend& _rend) {this->rendparam = _rend;}
  void setDetectionParameters(const apDetection& _detect) {this->detect = _detect;}
  void setSegmentationParameters(const apSegmentationParameters& _seg) {this->seg = _seg;}
  void setLearningParameters(const apLearn& _learn) {this->learn = _learn;}
  void setUseKalman(bool _useKalman){this->useKalman = _useKalman;}
  void setKalmanParameters(const apKalmanParameters& _kparam) {this->kalmanParameters = _kparam;}
  void setCCDParameters(const apCCDParameters& _ccdparams) {this->CCDParameters = _ccdparams;}
  void setKLTTrackerParameters(const apKLTTrackerParameters& _kltparams) {this->KLTTrackerParameters = _kltparams;}
  void setIprec(const vpImage<unsigned char> &I){Iprec = I;}
  void setIprecRGB(const vpImage<vpRGBa> &IRGB){IprecRGB = IRGB;}
  void setFrame(const int _frame){frame = _frame;}
  void loadConfigFile(const std::string& _filename);
  void loadConfigFile(const char* filename);
  void loadModel(const char* cad_name);
  void receiveImage(vpImage<vpRGBa> &Icol);
  void sendPose();
  void loadPointsNormals2d(std::vector<std::vector<point2d>> points2d, std::vector<std::vector<point2dd>> normals2d);
  void loadImagePoseMesh( cv::Mat &mat, vpHomogeneousMatrix &cMo, std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles);
  void loadImagePoseMeshControlPoints( cv::Mat &mat, vpHomogeneousMatrix &cMo, std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles);
  void savepair(std::string &message, const std::pair<point3d, point2d> &pair);
  void save3dpoint(std::string &message, const point3d &point3d);
  void save2dpoint(std::string &message, const point2d &point2d);
  //void loadLines(const vpImage<unsigned char>& I, vector<Vec4i>& Lines, const vpMatrix &Zc, const vpHomogeneousMatrix &cMo);
  //void resetLines();

  void initComm();
  void init(const vpImage<unsigned char>&){};
  void init(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo);
  void initKltTracker(const vpImage<unsigned char>& I);
  void initCircle(const vpPoint&, const vpPoint&, const vpPoint&, double, int, const string&){};
  //void initCylinder(const vpPoint&, vpPoint, double, unsigned int){};
  void initCylinder(const vpPoint&, const vpPoint&, double, int, const string&){};
  void track(const vpImage<unsigned char>& I, const vpImage<vpRGBa> &IRGB, const vpImage<vpRGBa> &Inormd,const vpImage<unsigned char>& Ior,const vpImage<unsigned char>& Itex, const double dist);
  //void trackHyb(const vpImage<unsigned char>& I, const vpImage<vpRGBa> &Inormd,const vpImage<unsigned char>& Ior,const vpImage<unsigned char>& Itex, const double dist, const double m, apOgre ogre_);
  void trackDef(const vpImage<unsigned char>& I, const vpImage<vpRGBa> &IRGB, const vpImage<vpRGBa> &Inormd,const vpImage<unsigned char>& Ior,const vpImage<unsigned char>& Itex, const double distmin, const double distmax);
  void trackDef2D(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB,  std::vector<point2d> &positions, std::vector<point2dd> &normals, std::vector<point2d> &trackededges, std::vector<int> &suppress);
  void trackPred(const vpImage<unsigned char> &I);
  void track(const vpImage<unsigned char>& I){};
  void trackXray(const vpImage<unsigned char>& I, double dist);
  void trackXrayIntensityContour(const vpImage<unsigned char>& I, const vpImage<vpRGBa> &IRGB, const vpImage<vpRGBa> &Inormd,const vpImage<unsigned char>& Ior,const vpImage<unsigned char>& Itex, const double dist);
  void track(const vpImage<unsigned char> &I, const vpImage<double> &Igrad, const vpImage<double> &Igradx, const vpImage<double> & Igrady, const vpImage<vpRGBa> &Inormd,const vpImage<unsigned char>& Ior,const vpImage<unsigned char>& Itex, const double dist);
  void display(const vpImage<unsigned char>&, const vpHomogeneousMatrix&, const vpCameraParameters&, const vpColor&, unsigned int, bool){};
  void display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor& col , const unsigned int l=1);
  void display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor& col , const unsigned int l=1);
  void display(const vpImage<vpRGBa>&, const vpHomogeneousMatrix &, const vpCameraParameters &, const vpColor&, unsigned int, bool){};
  void displayRend(const vpImage<vpRGBa>& I,  const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char> &Ior, const vpColor& col, const unsigned int thickness);
  void display(const vpImage<unsigned char>& I, vpColor col){};
  void displayKltPoints(const vpImage<unsigned char>& I);
  void sample(const vpImage<unsigned char>& I){};
  void resetTracker();
  void buildControlPoints2D(const vpImage<unsigned char> &I, std::vector<point2d> &points2d, std::vector<point2dd> &normals);
  void buildCorrespondencesEdges2D(std::vector<point2d> &trackededges, std::vector<int> &suppress);
  void exportCorrespondencesEdges2D(std::vector<std::vector<point2d>> &trackededgesIm, std::vector<std::vector<int>> &suppressIm);
  void exportCorrespondencesEdgesMean(const vpImage<unsigned char> &I);
  void exportCorrespondencesKLT(const vpImage<unsigned char> &I);
  //void reInitModel(const vpImage<unsigned char>& I, const char* cad_name, const vpHomogeneousMatrix& _cMo);
  //void reInitConfigModel(const vpImage<unsigned char>& I, const char* cad_name, const char* config_name, const vpHomogeneousMatrix& _cMo);

  /*!
    Enable to display the points along the control points with a color corresponding to their state.
    
    - If green : The vpMeSite is a good point.
    - If blue : The point is removed because of the vpMeSite tracking phase (constrast problem).
    - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
    - If red : The point is removed because of the robust method in the virtual visual servoing.
    
    \param displayMe : set it to true to display the points.
  */
  //void setDisplayControlPoints(const bool displayMe) {this->displayMe = displayMe;}
  
  /*!
    Set the first threshold used to check if the tracking failed. It corresponds to the percentage of good point which is necessary.
    
    The condition which has to be be satisfied is the following : \f$ nbGoodPoint > threshold1 \times (nbGoodPoint + nbBadPoint)\f$.
    
    The threshold is ideally between 0 and 1.
    
    \param threshold1 : The new value of the threshold. 
  */
  void setFirstThreshold(const double  threshold1) {percentageGdPt = threshold1;}
  /*!
    Get the moving edge parameters.
    
    \return an instance of the moving edge parameters used by the tracker.
  */
  inline void getMovingEdge(vpMe &_me ) { _me = this->me;}
  /*!
    Get the 3D rendering parameters.

    \return an instance of the rendering parameters used by the tracker and Ogre.
  */
  inline void getRendParameters(apRend &rend_ ) { rend_ = this->rendparam;}
  inline void getDetectionParameters(apDetection &detect_ ) { detect_ = this->detect;}
  inline void getSegmentationParameters(apSegmentationParameters &seg_ ) { seg_ = this->seg;}
  inline void getLearningParameters(apLearn &learn_ ) { learn_ = this->learn;}
  apTrackingType getTrackingType() { return this->trackingType;}
  bool getUseKalman(){ return useKalman;}
  bool getUseRGB(){ return useRGB;}
  vpHomogeneousMatrix getPoseVraie(){return cMoV;}
  inline void getKalmanParameters(apKalmanParameters &kparam_ ) { kparam_ = this->kalmanParameters;}
  inline void getCovarianceMatrix(vpMatrix &covMat_ ) { covMat_ = this->covarianceMatrixCCD;}
  inline void getCovarianceMatrixME(vpMatrix &covMat_ ) { covMat_ = this->covarianceMatrixME;}
  unsigned int getNbPoints(const unsigned int _level=0){return npoints;};
  unsigned int getNbPointsCCD(){return npointssil;}
  unsigned int getNbPointsKLT(){return nkltPoints;}
  std::vector<apControlPoint *>* getPpoint(const unsigned int _level);
  void setScales(const std::vector<bool>& _scales);
  void computeError(vpColVector &error);
  void computeError(vpColVector &error, vpHomogeneousMatrix &_cMo);
  void setGroundTruth(std::string input, std::string output, const int firstframe);
  void setTruePose(vpMatrix &truepose){truePose = truepose;}
  void setFirstFrame(int _firstFrame){firstFrame = _firstFrame;}

  
  /*!
    Return the scales levels used for the tracking. 
    
    \return The scales levels used for the tracking. 
  */
  std::vector<bool> getScales() const {return scales;}

 protected:
  void computeVVS(const vpImage<unsigned char>& _I);
  //void computeVVSHyb(const vpImage<unsigned char>& _I, apOgre ogre_);
  void computeVVSMH(const vpImage<unsigned char>& _I);
  void computeVVSPhotometric(const vpImage<unsigned char>& _I);
  void computeVVSCorr(const vpImage<unsigned char>& I, const vpImage<double>& Igrad, const vpImage<double>& Igradx, const vpImage<double>& Igrady);
  void computeVVSHybrid(const vpImage<unsigned char>& I, const vpImage<double>& Igrad, const vpImage<double>& Igradx, const vpImage<double>& Igrady);
  void computeVVSPointsLinesMH(const vpImage<unsigned char>& _I);
  void computeVVSPointsLinesRobustMH(const vpImage<unsigned char>& _I);


  void computeVVSCCD(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSCCDPrev(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSCCDMH(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSCCDMHPhotometric(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSCCDMHPrev(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSCCDMHPrevSpace(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSPointsLinesCCDMHPrev(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSPointsLinesCCDMHPrevSpace(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSCCDKltMHPrev(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSCCDKltMHPrevFAST(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);
  void computeVVSLinesCCDKltMHPrev(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB);

  void exportCorrespondencesEdges(const vpImage<unsigned char> &I);
  //void initControlPoints(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo) ;
  void trackControlPoints(const vpImage<unsigned char> &I);
  void trackControlPointsPred(const vpImage<unsigned char> &I);
  void trackControlPointsMH(const vpImage<unsigned char> &I);
  void trackControlPointsLinesMH(const vpImage<unsigned char> &I);
  void trackKltControlPoints(const vpImage<unsigned char> &I);
  void trackKltControlPointsFromSynth(const vpImage<unsigned char> &I, const vpImage<unsigned char> &Itex);
  void updateControlPoints();
  void extractControlPoints(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char> &Ior,const vpImage<unsigned char> &Itex, const double dist);
  void extractControlPoints(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char> &Ior,const vpImage<unsigned char> &Itex, const double distmin, const double distmax);
  void extractControlPointsP(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char>& Ior, const vpImage<unsigned char>& Itex, const double dist);
  void extractControlPointsCCD(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char> &Ior,const vpImage<unsigned char> &Itex, const double dist);
  void extractControlPointsLines(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char> &Ior,const vpImage<unsigned char> &Itex, const double dist);
  void extractControlPointsLinesCCD(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char> &Ior,const vpImage<unsigned char> &Itex, const double dist);
  void extractKltControlPoints(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd,const vpImage<unsigned char> &Ior,const vpImage<unsigned char> &Itex,const double dist);
  void extractKltControlPointsFAST(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd,const vpImage<unsigned char> &Ior,const vpImage<unsigned char> &Itex,const double dist);
  void loadLines(const vpImage<unsigned char>& I, vector<Vec4i>& Lines, const vpImage<vpRGBa>& Inormd, const vpHomogeneousMatrix &cMo);
  void addLine(const vpImage<unsigned char> &I, vpPoint &P1, vpPoint &P2, int polygone, std::string &name);
  void initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace = -1){};
  void initFaceFromCorners(vpMbtPolygon&){};
  void initFaceFromLines(vpMbtPolygon&){};
  void testTracking(){};
  vpColVector getError() const {};
  vpColVector getRobustWeights() const {};
  void computeVVSInit(){};
  void computeVVSInteractionMatrixAndResidu(){};
  void initPyramid(const vpImage<unsigned char>& _I, std::vector<const vpImage<unsigned char>* >& _pyramid);
  void initPyramid(const vpImage<vpRGBa>& _I, std::vector<const vpImage<vpRGBa>* >& _pyramid);
  void cleanPyramid(std::vector<const vpImage<unsigned char>* >& _pyramid);
  void cleanPyramid(std::vector<const vpImage<vpRGBa>* >& _pyramid);
  void reInitLevel(const unsigned int _lvl);
  void detectLandmarks(vpImage<unsigned char> &I, vpImage<vpRGBa> &Inormd);
  void initLandmarks(vpImage<unsigned char> &I, vpImage<vpRGBa> &Inormd);
  void loadLinesLm(const vpImage<unsigned char>& I, vector<Vec4i>& Lines, const vpImage<vpRGBa>& Inormd, const vpHomogeneousMatrix &cMo);
  void downScale(const unsigned int _scale);
  void upScale(const unsigned int _scale);
  vpMatrix computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b);
  vpMatrix computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b, const vpMatrix &W);
  vpMatrix computeCovarianceMatrix2(const vpMatrix &A, const vpColVector &x, const vpColVector &b, const vpMatrix &W);

  
};

#endif

